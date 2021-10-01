import json
import numpy as np
import os
import tensorflow as tf
from ring import generate_rings
_64 = 64

def reset_graph():
    if "sess" in globals() and sess:
        sess.close()
    tf.reset_default_graph()


class ConvVAE(object):
    def __init__(
        self,
        z_size=32,
        batch_size=1,
        learning_rate=0.0001,
        kl_tolerance=0.5,
        is_training=False,
        reuse=False,
        channels=1,
    ):
        self.z_size = z_size
        self.batch_size = batch_size
        self.learning_rate = learning_rate
        self.is_training = is_training
        self.kl_tolerance = kl_tolerance
        self.reuse = reuse
        self.channels = channels
        with tf.variable_scope("conv_vae", reuse=self.reuse):
            self._build_graph()
        self._init_session()

    def _build_graph(self):
        self.g = tf.Graph()
        with self.g.as_default():

            self.x = tf.placeholder(tf.float32, shape=[None, _64, _64, self.channels])

            # Encoder
            self.h1 = tf.layers.conv2d(
                self.x, 32, 4, strides=2, activation=tf.nn.relu, name="enc_conv1"
            )
            self.h2 = tf.layers.conv2d(
                self.h1, 64, 4, strides=2, activation=tf.nn.relu, name="enc_conv2"
            )
            self.h3 = tf.layers.conv2d(
                self.h2, 128, 4, strides=2, activation=tf.nn.relu, name="enc_conv3"
            )
            self.h4 = tf.layers.conv2d(
                self.h3, 256, 4, strides=2, activation=tf.nn.relu, name="enc_conv4"
            )
            self.h5 = tf.reshape(self.h4, [-1, 2 * 2 * 256])

            # VAE
            self.mu = tf.layers.dense(self.h5, self.z_size, name="enc_fc_mu")
            self.logvar = tf.layers.dense(self.h5, self.z_size, name="enc_fc_log_var")
            self.sigma = tf.exp(self.logvar / 2.0)
            self.epsilon = tf.random_normal([self.batch_size, self.z_size])
            if self.is_training:
                self.z = self.mu + self.sigma * self.epsilon
            else:
                self.z = self.mu

            # Decoder
            self.h6 = tf.layers.dense(self.z, 4 * 256, name="dec_fc")
            self.h6 = tf.reshape(self.h6, [-1, 1, 1, 4 * 256])
            self.h7 = tf.layers.conv2d_transpose(
                self.h6, 128, 5, strides=2, activation=tf.nn.relu, name="dec_deconv1"
            )
            self.h8 = tf.layers.conv2d_transpose(
                self.h7, 64, 5, strides=2, activation=tf.nn.relu, name="dec_deconv2"
            )
            self.h9 = tf.layers.conv2d_transpose(
                self.h8, 32, 6, strides=2, activation=tf.nn.relu, name="dec_deconv3"
            )
            self.y = tf.layers.conv2d_transpose(
                self.h9,
                self.channels,
                6,
                strides=2,
                activation=tf.nn.sigmoid,
                name="dec_deconv4",
            )

            # train ops
            if self.is_training:
                self.global_step = tf.Variable(0, name="global_step", trainable=False)

                eps = 1e-6  # avoid taking log of zero

                # reconstruction loss
                self.r_loss = tf.reduce_sum(
                    tf.square(self.x - self.y), reduction_indices=[1, 2, 3]
                )
                self.r_loss = tf.reduce_mean(self.r_loss)

                # augmented kl loss per dim
                self.kl_loss = -0.5 * tf.reduce_sum(
                    (1 + self.logvar - tf.square(self.mu) - tf.exp(self.logvar)),
                    reduction_indices=1,
                )
                self.kl_loss = tf.maximum(self.kl_loss, self.kl_tolerance * self.z_size)
                self.kl_loss = tf.reduce_mean(self.kl_loss)

                self.loss = self.r_loss + self.kl_loss

                # training
                self.lr = tf.Variable(self.learning_rate, trainable=False)
                self.optimizer = tf.train.AdamOptimizer(self.lr)
                grads = self.optimizer.compute_gradients(
                    self.loss
                )  # can potentially clip gradients here.

                self.train_op = self.optimizer.apply_gradients(
                    grads, global_step=self.global_step, name="train_step"
                )

            # initialize vars
            self.init = tf.global_variables_initializer()

            t_vars = tf.trainable_variables()
            self.assign_ops = {}
            for var in t_vars:
                # if var.name.startswith('conv_vae'):
                pshape = var.get_shape()
                pl = tf.placeholder(tf.float32, pshape, var.name[:-2] + "_placeholder")
                assign_op = var.assign(pl)
                self.assign_ops[var] = (assign_op, pl)

    def _init_session(self):
        """Launch TensorFlow session and initialize variables"""
        self.sess = tf.Session(graph=self.g)
        self.sess.run(self.init)

    def close_sess(self):
        """ Close TensorFlow session """
        self.sess.close()

    def encode(self, x):
        return self.sess.run(self.z, feed_dict={self.x: x})

    def encode_mu_logvar(self, x):
        (mu, logvar) = self.sess.run([self.mu, self.logvar], feed_dict={self.x: x})
        return mu, logvar

    def decode(self, z):
        return self.sess.run(self.y, feed_dict={self.z: z})

    def encode_decode(self, x):
        return self.sess.run(self.y, feed_dict={self.x: x})
    def encode_decode_debug(self, x):
        # return self.sess.run()
        return self.sess.run([self.h1,self.h2,self.h3,self.h4,self.h5,self.h6,self.h7,self.h8,self.h9,self.mu], feed_dict={self.x:x})


    def get_model_params(self):
        # get trainable params.
        model_names = []
        model_params = []
        model_shapes = []
        with self.g.as_default():
            t_vars = tf.trainable_variables()
            for var in t_vars:
                # if var.name.startswith('conv_vae'):
                param_name = var.name
                p = self.sess.run(var)
                model_names.append(param_name)
                params = np.round(p * 10000).astype(np.int).tolist()
                model_params.append(params)
                model_shapes.append(p.shape)
        return model_params, model_shapes, model_names

    def print_trainable_params(self):
        trainable_params = 0
        with self.g.as_default():
            t_vars = tf.trainable_variables()
            for var in t_vars:
                trainable_params += np.prod(var.get_shape().as_list())
        print("VAE Trainable parameters: {}".format(trainable_params))

    def get_random_model_params(self, stdev=0.5):
        # get random params.
        _, mshape, _ = self.get_model_params()
        rparam = []
        for s in mshape:
            # rparam.append(np.random.randn(*s)*stdev)
            rparam.append(np.random.standard_cauchy(s) * stdev)  # spice things up
        return rparam

    def set_model_params(self, params):
        with self.g.as_default():
            t_vars = tf.trainable_variables()
            idx = 0
            for var in t_vars:
                # if var.name.startswith('conv_vae'):
                pshape = tuple(var.get_shape().as_list())
                p = np.array(params[idx])
                assert pshape == p.shape, "inconsistent shape"
                assign_op, pl = self.assign_ops[var]
                self.sess.run(assign_op, feed_dict={pl.name: p / 10000.0})
                idx += 1

    def load_json(self, jsonfile="vae.json"):
        with open(jsonfile, "r") as f:
            params = json.load(f)
        self.set_model_params(params)

    def save_json(self, jsonfile="vae.json"):
        model_params, model_shapes, model_names = self.get_model_params()
        qparams = []
        for p in model_params:
            qparams.append(p)
        with open(jsonfile, "wt") as outfile:
            json.dump(
                qparams, outfile, sort_keys=True, indent=0, separators=(",", ": ")
            )

    def set_random_params(self, stdev=0.5):
        rparam = self.get_random_model_params(stdev)
        self.set_model_params(rparam)

    def save_model(self, model_save_path):
        sess = self.sess
        with self.g.as_default():
            saver = tf.train.Saver(tf.global_variables())
        checkpoint_path = os.path.join(model_save_path, "vae")
        tf.logging.info("saving model %s.", checkpoint_path)
        saver.save(sess, checkpoint_path, 0)  # just keep one

    def load_checkpoint(self, checkpoint_path):
        sess = self.sess
        with self.g.as_default():
            saver = tf.train.Saver(tf.global_variables())
        ckpt = tf.train.get_checkpoint_state(checkpoint_path)
        print("loading model", ckpt.model_checkpoint_path)
        tf.logging.info("Loading model %s.", ckpt.model_checkpoint_path)
        saver.restore(sess, ckpt.model_checkpoint_path)


if __name__ == "__main__":
    from matplotlib import pyplot as plt


    vae = ConvVAE()
    vae.print_trainable_params()
    json_filepath_relative = "vae.json"
    dir_path = os.path.dirname(__file__)
    json_filepath = os.path.join(dir_path, json_filepath_relative)
    vae.load_json(json_filepath)



    ring_def = generate_rings()
    # linear ramp
    scan = np.ones((1, 1080)) * (np.arange(1080) / 1080.0 * 25.0)[None, :]
    ring_scan =  ring_def["lidar_to_rings"](scan.astype(np.float32))/ring_def["rings_to_bool"]

    # ring_scan_decode = vae.encode_decode(ring_scan)

    # plt.figure("training_status")
    # plt.clf()
    # f, (ax1, ax2) = plt.subplots(2, 1, num="training_status")
    # ax1.imshow(ring_scan[0, :, :, 0], cmap=plt.cm.Greys)
    # ax2.imshow(vae.encode_decode(ring_scan)[0, :, :, 0], cmap=plt.cm.Greys)
    # plt.savefig("compare.png")
    
    h1,h2,h3,h4,h5,h6,h7,h8,h9,mu = vae.encode_decode_debug(ring_scan)
    np.savez("tf",e1=h1,e2=h2,e3=h3,e4=h4,e5=h5,f3=h6,d1=h7,d2=h8,d3=h9,f1=mu)