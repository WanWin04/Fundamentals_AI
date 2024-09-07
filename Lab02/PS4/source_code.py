from os import walk, mkdir
from os.path import exists, join


global resolvents


def get_files(directory_path):
    lists = []
    for (dirpath, dirnames, filenames) in walk(directory_path):
        lists.extend(filenames)
    return lists


def analysis_clause(clause):
    return clause.replace('\n', '').replace(' ', '').split('OR')


def normalize_clause(clause):
    clause = sorted(list(set(clause)), key=lambda x: x[-1])
    for i in range(len(clause) - 1):
        if clause[i][-1] == clause[i + 1][-1]:
            return [True]
    return clause


def resolve_clauses(clause1, clause2):
    negative_literals = []
    for index1 in clause1:
        for index2 in clause2:
            if index1[-1] == index2[-1] and len(index1) != len(index2):
                negative_literals.append(index1[-1])
                
    if len(negative_literals) == 1:
        resolved_clause = list(set(clause1) | set(clause2))
        resolved_clause.remove(negative_literals[0])
        resolved_clause.remove('-' + negative_literals[0])
        return sorted(resolved_clause, key=lambda x: x[-1])
    
    return [True]


def PL_resolution(knowledge_base, query):
    global resolvents
    resolvents = []
    clauses = []
    
    for clause in knowledge_base:
        normalized_clause = normalize_clause(analysis_clause(clause))
        clauses.append(normalized_clause)
        
    normalized_query = normalize_clause(analysis_clause(query))
    if normalized_query == [True]:
        return True
    
    for literal in normalized_query:
        if len(literal) == 1:
            clauses.append(['-' + literal])
        else:
            clauses.append([literal[-1]])
            
    iteration = 0
    while True:
        iteration += 1
        
        is_empty_clause = False
        new_clauses = []
        for clause1 in clauses:
            for clause2 in clauses:
                resolved = resolve_clauses(clause1, clause2)
                if len(resolved) == 0:
                    is_empty_clause = True
                    if ['{}'] not in new_clauses:
                        new_clauses.append(['{}'])
                elif resolved == [True]:
                    continue
                else:
                    if resolved not in clauses and resolved not in new_clauses:
                        new_clauses.append(resolved)
        
        resolvents.append([clause for clause in new_clauses if clause not in clauses])
        
        if is_empty_clause:
            return True
        if all(clause in clauses for clause in new_clauses):
            return False
        
        clauses.extend(new_clauses)


def main():
    INPUT_DIR = "../Input/"
    OUTPUT_DIR = "../Output/"

    if not exists(OUTPUT_DIR):
        mkdir(OUTPUT_DIR)

    input_files = get_files(INPUT_DIR)

    for input_file in input_files:
        input_path = join(INPUT_DIR, input_file)
        output_file_name = f"output{input_file.replace('input', '')}"
        output_path = join(OUTPUT_DIR, output_file_name)

        with open(input_path, 'r') as file:
            lines = file.readlines()

        query = lines[0].strip()
        num_clauses = int(lines[1].strip())
        knowledge_base = [line.strip() for line in lines[2:2 + num_clauses]]

        entails = PL_resolution(knowledge_base, query)

        with open(output_path, 'w') as file:
            for i, new_clauses in enumerate(resolvents):
                file.write(f"{len(new_clauses)}\n")
                for clause in new_clauses:
                    file.write(" OR ".join(clause) + "\n")

            file.write("YES" if entails else "NO")


if __name__ == "__main__":
    main()
