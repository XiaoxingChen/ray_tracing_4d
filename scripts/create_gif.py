import imageio

if __name__ == "__main__":
    with imageio.get_writer('build/mygif.gif', mode='I', duration=0.05) as writer:
        for i in range(64):
            filename = "image_4d_{}.ppm".format(i)
            image = imageio.imread(filename)
            writer.append_data(image)