import argparse

class FileManager:
    def __init__(self, raw_filename):
        self.raw_filename = raw_filename

        parser = argparse.ArgumentParser()
        parser.add_argument("-o", "--output", help="Output file path")
        parser.add_argument("-d", "--directory", help="Destination directory for default output filename")
        self.args = parser.parse_args()

    def get_output_file_path(self):
        args = self.args

        output_file_path = ""
        if args.output is None and args.directory is None:
            # dump in current directory
            output_file_path = self.raw_filename
        elif args.output:
            output_file_path = args.output
        elif args.directory:
            output_file_path = args.directory + "/" + self.raw_filename

        return output_file_path
