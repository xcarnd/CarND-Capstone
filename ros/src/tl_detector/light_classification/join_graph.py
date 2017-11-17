import sys
import os

GRAPH_NAME = sys.argv[1]

if __name__ == '__main__':
    with open(GRAPH_NAME + ".pb", 'wb') as gf:
        cnt = 0
        part_file = os.path.join(GRAPH_NAME,
                                 "{}.parts.{}".format(GRAPH_NAME, cnt))
        while os.path.exists(part_file):
            with open(part_file, 'rb') as p:
                chunk = p.read()
                gf.write(chunk)
            cnt += 1
            part_file = os.path.join(GRAPH_NAME,
                                     "{}.parts.{}".format(GRAPH_NAME, cnt))
