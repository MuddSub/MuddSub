''' __main__.py
The main module for netdev package.
'''
import argparse
from netdev.find_anchors import find_anchors
from netdev.arch_test import arch_test


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    # subparser argument
    subparsers = parser.add_subparsers(title='command', dest='command')
    # subparser for anchor finding
    F_parser = subparsers.add_parser('F', help='anchor finding')
    F_parser.add_argument('data_path', type=str,
        help='the path to the raw data, where labels are all stored in a .txt format')
    F_parser.add_argument('input_dim', type=int,
        help='the input dimension of images to the network')
    F_parser.add_argument('num_anchors', type=int,
        help='the number of anchors that the network should have (total)')
    # subparser for architecture testing
    T_parser = subparsers.add_parser('T', help='architecture testing')
    T_parser.add_argument('cfg_path',
        help='the relative path to the config (.cfg) file to test')
    T_parser.add_argument('--batch_size', dest='batch_size', default=32,
        type=int, help='the batch size (default: 32)')
    T_parser.add_argument('--stop_at', dest='stop_at', default=None,
        type=int, help='after which layer to stop the testing loop, if desired')
    # parse arguments
    args = parser.parse_args()

    # run the command
    if args.command == 'F':
        find_anchors(args.data_path, args.input_dim, args.num_anchors)
    elif args.command == 'T':
        # architecture testing
        arch_test(args.cfg_path, args.batch_size, args.stop_at)
