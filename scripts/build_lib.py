import sys
import os
import shutil
import argparse

class WebDrive():
    @staticmethod
    def wget(link, folder):
        os.system("wget {} --directory-prefix={} --no-clobber".format(link, folder))

    @staticmethod
    def sync(yaml_file, output_folder):
        from multiprocessing import Pool
        import yaml

        with open(yaml_file) as f:
            input_dict = yaml.safe_load(f)
            renaming_urls = {}
            if 'renamings' in input_dict and input_dict['renamings'] is not None:
                renaming_urls = input_dict['renamings']
            renamingless_urls = []
            if 'urls' in input_dict and input_dict['urls'] is not None:
                renamingless_urls = input_dict['urls']

        url_list = []
        rename_target = []
        for name in renaming_urls.keys():
            if os.path.isfile(os.path.join(output_folder, name)):
                print("{} exist, not retrieving".format(name))
                continue
            url_list += renaming_urls[name]
            rename_target.append(name)

        url_list += renamingless_urls

        params = [(link, output_folder) for link in url_list]

        os.makedirs(output_folder, exist_ok=True)
        with Pool(10) as p:
            p.starmap(WebDrive.wget, params)

        for name in rename_target:
            input_files = [os.path.join(output_folder, os.path.basename(ifile)) for ifile in renaming_urls[name]]

            output_file = os.path.join(output_folder, name)
            WebDrive.merge(output_file, input_files)

    @staticmethod
    def merge(output_name, input_files):
        for input_file in input_files:
            if not os.path.isfile(input_file):
                print("{} not exists, merge failed".format(input_file))
                # print(input_files)
                return

        print("write merged file to " + output_name)
        if len(input_files) == 1:
            shutil.move(input_files[0], output_name)
            return

        with open(output_name, 'wb') as of:
            for input_file in input_files:
                with open(input_file, 'rb') as f:
                    of.write(f.read())
                os.remove(input_file)

def handleAutoComplete():
    if sys.platform == 'linux':
        complete_cmd = 'complete -F _longopt {}'.format(os.path.basename(__file__))
        bashrc_path = os.path.expanduser('~/.bashrc')
        with open(bashrc_path) as f:
            if not complete_cmd in f.read():
                os.system('echo "{}" >> {}'.format(complete_cmd, bashrc_path))
    else:
        pass