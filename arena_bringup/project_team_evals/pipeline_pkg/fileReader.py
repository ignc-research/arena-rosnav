import os
from argparse import ArgumentParser

class Reader:
    def read(args):
        rootdir = args.root_dir
        image_path = args.image_path
        for dir in os.listdir(rootdir):
            print("")
            print("Currently working on fileName",dir)
            print("")
            if(dir != "CSVaverages" and not(os.path.isdir(dir))):
                os.system("python3 createAverage.py --csv_path {} --csv_name /{}/{}*csv --image_path {}".format(rootdir,dir,dir,image_path))

if __name__ == "__main__":
    dirname = os.path.dirname(os.path.abspath(__file__))
    image_path = "{}/maps".format(dirname) 
    root_path = "{}/sims_data_records".format(dirname)

    parser = ArgumentParser()
    parser.add_argument(
        "--image_path",
        action="store",
        dest="image_path",
        default=image_path,
        help="path to the floor plan of your world. Usually in .pgm format",
        required=False,
    )
    parser.add_argument(
        "--root_dir",
        action="store",
        dest="root_dir",
        default=root_path,
        help="path to the dir containing the directorys with the csv files file you want to use as input",
        required=False,
    )
    args = parser.parse_args()
    Reader.read(args)