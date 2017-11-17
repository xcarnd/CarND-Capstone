import sys
import os

GRAPH_FILE = sys.argv[1]
CHUNK_SIZE = 10 * 1024 * 1024

if __name__ == '__main__':
    graph_name = GRAPH_FILE.split(".")[0]
    if not os.path.exists(graph_name):
        os.makedirs(graph_name)
    with open(GRAPH_FILE, 'rb') as gf:
        cnt = 0
        chunk = gf.read(CHUNK_SIZE)
        while chunk:
            with open(
                    os.path.join(
                        graph_name,
                        "{}.parts.{}".format(graph_name, cnt)), 'wb') as p:
                p.write(chunk)
            chunk = gf.read(CHUNK_SIZE)
            cnt += 1
