import imageio

if __name__ == "__main__":
    with imageio.get_writer('build/mygif.gif', mode='I') as writer:
        for i in range(15):
            filename = "exercise_5_4d_{}.ppm".format(i)
            image = imageio.imread(filename)
            writer.append_data(image)