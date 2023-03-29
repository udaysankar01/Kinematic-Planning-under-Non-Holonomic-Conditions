import os
import cv2
import imageio
from planner import Planner
from world import World
import matplotlib.pyplot as plt
import argparse, textwrap

def main():

    parser = argparse.ArgumentParser(description='Valet Assignment by Uday Sankar',
        usage='use "python %(prog)s --help" for more information',
        formatter_class=argparse.RawTextHelpFormatter)
    
    parser.add_argument('robotType', default=1, type=int,
        help= textwrap.dedent('''\
        Delivery Robot: 1
        Car: 2
        Truck with Trailer: 3

        default: 1
        '''))
    
    args = parser.parse_args()
    robotType = args.robotType
    
    animation_folder = './animations'
    path_depiciton_folder = './paths'
    if not os.path.exists(animation_folder):
        os.makedirs(animation_folder)
    if not os.path.exists(path_depiciton_folder):
        os.makedirs(path_depiciton_folder)

    if robotType == 1:
        gifname = f'{animation_folder}/delivery_robot.gif'
        pathname = f'{path_depiciton_folder}/delivery_robot.png'

    elif robotType == 2:
        gifname = f'{animation_folder}/car.gif'
        pathname = f'{path_depiciton_folder}/car.png'
    
    elif robotType == 3:
        gifname = f'{animation_folder}/truck.gif'
        pathname = f'{path_depiciton_folder}/truck.png'
    
    else:
        print('Wrong robot type chosen!')
        quit()

    world = World(robotType)
    pathPlanner = Planner(world, robotType)
    path = pathPlanner.A_star()

    images = []
    if robotType != 3:
        for i, point in enumerate(path):
            img = world.visualizeWorld(point[0],point[1],point[2])
            images.append(img)
    else:
        for i, point in enumerate(path):
            img = world.visualizeWorld([point[0],point[3]], [point[1], point[4]], [point[2],point[5]])
            images.append(img)
    
    imageio.mimsave(gifname, images, fps = 20)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    plt.figure("Axle Path")
    plt.xlim([0, 200])
    if robotType != 3:
        plt.ylim([-10, 160])
    else:
        plt.ylim([-10, 200])
    for i, point in enumerate(path):
        plt.scatter(point[0], point[1], color='gray')
    plt.savefig(pathname)


if __name__ == "__main__":
    main()