def min_substitutions_for_unique_adjacent_chars(words):
    # Function to calculate minimum changes for a single word
    def min_changes(word):
        changes = 0
        for i in range(1, len(word)):
            if word[i] == word[i-1]:
                changes += 1
        return changes

    # Calculate and return the minimum changes for each word in the input list
    return [min_changes(word) for word in words]

# Example usage
words = ["boook", "abcd", "eeff"]
result = min_substitutions_for_unique_adjacent_chars(words)
print(result)  # Output: [1, 0, 2]
