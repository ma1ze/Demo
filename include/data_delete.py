
import os
import glob

# folders location


#删除文件夹下面的所有文件(只删除文件,不删除文件夹)
import os
import shutil

def del_files(path):
    for root, dirs, files in os.walk(path):
        for name in files:
            if name.endswith(".png"):
                os.remove(os.path.join(root, name))


            if name.endswith(".yaml"):
                os.remove(os.path.join(root, name))
                print("Delete File: " + os.path.join(root, name))

    print("files are deleted")


# test
if __name__ == "__main__":
    path = '/home/dong/PycharmProjects/testDemo/images/calib'
    del_files(path)
