from Animations import run
import sys

if __name__ == '__main__':
    """
        Argument from command line: `python main.py <input_file_path> <algorithm> <time_delay>(optional)`
        search_algorithm must be one of ['bfs', 'dfs', 'ucs', 'greedy', 'astar']
    """
    if len(sys.argv) < 3 or len(sys.argv) > 5:
        raise ValueError("Wrong input!!!")
    
    input_file_path = str(sys.argv[1])
    algorithm = str(sys.argv[2])
    
    if algorithm not in ['bfs', 'dfs', 'ucs', 'greedy', 'astar']:
        raise ValueError("Invalid algorithm!!!")
    
    time_delay = int(sys.argv[3]) if len(sys.argv) == 4 else 500
    
    run(input_file_path, algorithm, time_delay)
    
