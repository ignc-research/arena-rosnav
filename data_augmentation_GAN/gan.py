
import matplotlib.pyplot as plt
import tensorflow as tf
import numpy as np
from tensorflow.keras import layers
from IPython import display
import time
import os
from argparse import ArgumentParser


class Gan:
    def getData(self, path):
        img = tf.io.read_file(path)
        img = tf.image.decode_jpeg(img, channels=3)
        img = tf.image.convert_image_dtype(img, tf.float32)
        img = tf.divide(tf.subtract(tf.multiply(img, 255), 127.5), 127.5)
        return tf.image.resize(img, (128, 128))

    def generator(self):
        model = tf.keras.models.Sequential()
        model.add(
            layers.Dense(16 * 16 * 512, input_shape=(1,), use_bias=False)
        )  # creates output shape of 7,7 with number of neurons - [7,7,256]
        model.add(layers.BatchNormalization())
        model.add(layers.LeakyReLU())

        model.add(layers.Reshape((16, 16, 512)))
        assert model.output_shape == (None, 16, 16, 512)

        model.add(
            layers.Conv2DTranspose(
                256, (5, 5), strides=(1, 1), padding="same", use_bias=False
            )
        )  # stride (1,1) keeps the same shape as that of input
        assert model.output_shape == (None, 16, 16, 256)
        model.add(layers.BatchNormalization())
        model.add(layers.LeakyReLU())
        ## output of shape (7,7,128)

        model.add(
            layers.Conv2DTranspose(
                128, (4, 4), strides=(2, 2), padding="same", use_bias=False
            )
        )  # stride (2,2) doubles the size of the input
        assert model.output_shape == (None, 32, 32, 128)
        model.add(layers.BatchNormalization())
        model.add(layers.LeakyReLU())
        ## output shape (14,14,64)

        model.add(
            layers.Conv2DTranspose(
                128, (4, 4), strides=(2, 2), padding="same", use_bias=False
            )
        )  # stride (2,2) doubles the size of the input
        assert model.output_shape == (None, 64, 64, 128)
        model.add(layers.BatchNormalization())
        model.add(layers.LeakyReLU())

        model.add(
            layers.Conv2DTranspose(
                3,
                (4, 4),
                strides=(2, 2),
                padding="same",
                activation="tanh",
                use_bias=False,
            )
        )
        assert model.output_shape == (None, 128, 128, 3)
        ## output shape (28,28,1) the required shape

        return model

    def discriminator(self):
        model = tf.keras.models.Sequential()
        model.add(
            layers.Conv2D(
                128, (5, 5), strides=(2, 2), padding="same", input_shape=[128, 128, 3]
            )
        )
        model.add(layers.LeakyReLU())
        model.add(layers.Dropout(0.3))

        model.add(layers.Conv2D(256, (4, 4), strides=(2, 2), padding="same"))
        model.add(layers.LeakyReLU())
        model.add(layers.Dropout(0.3))
        model.add(
            layers.Conv2D(
                256, (5, 5), strides=(2, 2), padding="same"
            )
        )
        model.add(layers.Flatten())
        model.add(layers.Dense(1))

        return model

    def lossDis(self, real_output, fake_output, crossEntropy):
        real_loss = crossEntropy(tf.ones_like(real_output), real_output)
        fake_loss = crossEntropy(tf.zeros_like(fake_output), fake_output)
        total_loss = real_loss + fake_loss
        return total_loss

    def lossGen(self, fake_output, crossEntropy):
        return crossEntropy(tf.ones_like(fake_output), fake_output)

    @tf.function
    def trainStep(
        self,
        images,
        generatorModel,
        discriminatorModel,
        crossEntropy,
        BATCH_SIZE,
        noise_dims,
        generator_optimizer,
        discriminator_optimizer,
    ):
        noise = tf.random.normal([BATCH_SIZE, noise_dims])

        with tf.GradientTape() as gen_tape, tf.GradientTape() as dis_tape:
            generated_images = generatorModel(noise, training=True)

            real_output = discriminatorModel(images, training=True)
            fake_output = discriminatorModel(generated_images, training=True)

            gen_loss = self.lossGen(fake_output, crossEntropy)
            disc_loss = self.lossDis(real_output, fake_output, crossEntropy)

        gen_gradients = gen_tape.gradient(gen_loss, generatorModel.trainable_variables)
        dis_gradients = dis_tape.gradient(
            disc_loss, discriminatorModel.trainable_variables
        )

        generator_optimizer.apply_gradients(
            zip(gen_gradients, generatorModel.trainable_variables)
        )
        discriminator_optimizer.apply_gradients(
            zip(dis_gradients, discriminatorModel.trainable_variables)
        )

    def training(
        self,
        dataset,
        epochs,
        generatorModel,
        discriminatorModel,
        crossEntropy,
        checkpoint_prefix,
        checkpoint,
        seed,
        BATCH_SIZE,
        noise_dims,
        generator_optimizer,
        discriminator_optimizer,
        output,
    ):
        for epoch in range(epochs):
            start = time.time()
            for batch in dataset:
                self.trainStep(
                    batch,
                    generatorModel,
                    discriminatorModel,
                    crossEntropy,
                    BATCH_SIZE,
                    noise_dims,
                    generator_optimizer,
                    discriminator_optimizer,
                )
            display.clear_output(wait=True)
            self.save_output(generatorModel, epoch + 1, seed, output)

            if (epoch + 1) % 15 == 0:
                checkpoint.save(file_prefix=checkpoint_prefix)

            print(f"Generating time for map {epoch + 1} is {time.time()-start}")

        display.clear_output(wait=True)
        self.save_output(generatorModel, epochs, seed, output)

    def save_output(self, model, epoch, test_input, output):
        predictions = model(test_input, training=False)
        # predictions = predictions.numpy().reshape(16,64,64,1)
        fig = plt.figure(figsize=(1.5, 1.5))
        # print(predictions)
        plt.gca().set_axis_off()
        plt.subplots_adjust(top = 1, bottom = 0, right = 1, left = 0,
                    hspace = 1, wspace = 1)
        # plt.margins(0,0)
        # plt.gca().xaxis.set_major_locator(plt.NullLocator())
        # plt.gca().yaxis.set_major_locator(plt.NullLocator())
        for i in range(predictions.shape[0]):
            # plt.subplot(4,4,i+1)
            plt.imshow(
                (predictions[i] * 127.5 + 127.5).numpy().astype(np.uint8), cmap="gray"
            )
            plt.axis("off")
        path = os.path.join(output, f"map_{epoch}.png")
        plt.savefig(path)
        # plt.show()


def main():

    parser = ArgumentParser()
    parser.add_argument(
        "--image_path",
        action="store",
        dest="image_path",
        default=None,
        help="path to the floor plan of your world. Usually in .png format",
        required=True,
    )
    parser.add_argument(
        "--output_path",
        action="store",
        dest="output_path",
        default=None,
        help="location to store the generated images.",
        required=True,
    )
    args = parser.parse_args()

    images = []
    for i in os.scandir(args.image_path):
        images.append(i.path)

    images = tf.data.Dataset.from_tensor_slices(images)
    BATCH_SIZE = 5
    train_images = (
        images.map(Gan().getData, num_parallel_calls=tf.data.experimental.AUTOTUNE)
        .batch(BATCH_SIZE)
        .shuffle(60000)
    )

    generatorModel = Gan().generator()
    noise = tf.random.normal([1, 1])
    generated_image = generatorModel(noise, training=False)

    # x = generated_image.numpy().reshape(64,64,1)
    plt.imshow(generated_image[0] * 127.5 + 127.5)

    discriminatorModel = Gan().discriminator()
    decision = discriminatorModel(generated_image)
    print("----------------->", decision)

    crossEntropy = tf.keras.losses.BinaryCrossentropy(from_logits=True)
   #  crossEntropy = tf.keras.losses.MeanSquaredError(
   #  name='mean_squared_error'
   #  )


    generator_optimizer = tf.keras.optimizers.Adam(1e-4)
    discriminator_optimizer = tf.keras.optimizers.Adam(1e-4)

    checkpoint_dir = "./training_checkpoints"
    checkpoint_prefix = os.path.join(checkpoint_dir, "ckpt")
    checkpoint = tf.train.Checkpoint(
        generator_optimizer=generator_optimizer,
        discriminator_optimizer=discriminator_optimizer,
        generator=generatorModel,
        discriminator=discriminatorModel,
    )

    results_dir = "output/"

    EPOCHS = 200
    noise_dims = 1
    num_egs_to_generate = 1
    seed = tf.random.normal([num_egs_to_generate, noise_dims])

    Gan().training(
        train_images,
        EPOCHS,
        generatorModel,
        discriminatorModel,
        crossEntropy,
        checkpoint_prefix,
        checkpoint,
        seed,
        BATCH_SIZE,
        noise_dims,
        generator_optimizer,
        discriminator_optimizer,
        args.output_path,
    )

    while True:
        new_image = generatorModel(tf.random.normal([1, 128]), training=False)
        plt.imshow(new_image[0, :, :, :])
        plt.axis("off")
        plt.show()


if __name__ == "__main__":
    main()

# Argument example
# --image_path
# /home/nilou/Schreibtisch/generated_maps/images
# --output_path
# /home/nilou/Schreibtisch/git/IAS_Naviprediction/data_augmentation_GAN/example/output
