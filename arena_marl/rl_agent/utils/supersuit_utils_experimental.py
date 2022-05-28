class SharedArray_patched(SharedArray):
    def __setstate__(self, state):
        (self.shared_arr, self.dtype, self.shape) = state
        self._set_np_arr()

class ProcConcatVec_patched(ProcConcatVec):
    def __init__(self, vec_env_constrs, observation_space, action_space, tot_num_envs, metadata):
        self.observation_space = observation_space
        self.action_space = action_space
        self.num_envs = num_envs = tot_num_envs
        self.metadata = metadata

        self.shared_obs = SharedArray_patched((num_envs,) + self.observation_space.shape, dtype=self.observation_space.dtype)
        act_space_wrap = SpaceWrapper(self.action_space)
        self.shared_act = SharedArray_patched((num_envs,) + act_space_wrap.shape, dtype=act_space_wrap.dtype)
        self.shared_rews = SharedArray_patched((num_envs,), dtype=np.float32)
        self.shared_dones = SharedArray_patched((num_envs,), dtype=np.uint8)

        pipes = []
        procs = []
        for constr in vec_env_constrs:
            inpt, outpt = mp.Pipe()
            constr = gym.vector.async_vector_env.CloudpickleWrapper(constr)
            proc = mp.Process(
                target=async_loop, args=(constr, inpt, outpt, self.shared_obs, self.shared_act, self.shared_rews, self.shared_dones)
            )
            proc.start()
            outpt.close()
            pipes.append(inpt)
            procs.append(proc)

        self.pipes = pipes
        self.procs = procs

        num_envs = 0
        env_nums = self._receive_info()
        idx_starts = []
        for pipe, cnum_env in zip(self.pipes, env_nums):
            cur_env_idx = num_envs
            num_envs += cnum_env
            pipe.send(cur_env_idx)
            idx_starts.append(cur_env_idx)

        assert num_envs == tot_num_envs, f"num_envs={num_envs} und tot_num_envs={tot_num_envs}"
        self.idx_starts = idx_starts

class call_wrap_patched:
    def __init__(self, fn, data, i):
        self.fn = fn
        self.data = data
        self.i = i

    def __call__(self, *args):
        rospy.init_node(f"train_env_{self.i}", disable_signals=False, anonymous=True)
        return self.fn(self.data)

def MakeCPUAsyncConstructor_patched(max_num_cpus, metadata):
    if True:
        rospy.init_node("train_env", disable_signals=False, anonymous=True)
        return ConcatVecEnv
    else:

        def constructor(env_fn_list, obs_space, act_space):
            num_fns = len(env_fn_list)
            envs_per_cpu = (num_fns + max_num_cpus - 1) // max_num_cpus
            alloced_num_cpus = (num_fns + envs_per_cpu - 1) // envs_per_cpu

            env_cpu_div = []
            num_envs_alloced = 0
            while num_envs_alloced < num_fns:
                start_idx = num_envs_alloced
                end_idx = min(num_fns, start_idx + envs_per_cpu)
                env_cpu_div.append(env_fn_list[start_idx:end_idx])
                num_envs_alloced = end_idx

            assert alloced_num_cpus == len(env_cpu_div)

            cat_env_fns = [call_wrap_patched(ConcatVecEnv, env_fns, i) for i, env_fns in enumerate(env_cpu_div)]
            return ProcConcatVec_patched(cat_env_fns, obs_space, act_space, num_fns, metadata)

        return constructor