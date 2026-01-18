
import argparse
from lmfit_optimizer import lmfit_main
from scipy_optimizer import scipy_optimize_main


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--optimizer", default='scipy')
    args = parser.parse_args()

    if args.optimizer == 'lmfit':
        lmfit_main()
    else:
        scipy_optimize_main()

if __name__ == "__main__":
    main()
