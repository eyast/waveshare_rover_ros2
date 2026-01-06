import random

# First pass: count lines
with open('accelerator_samples_old.txt', 'r') as f:
    total_lines = sum(1 for _ in f)

# Calculate how many to sample
sample_size = max(1, total_lines // 100)

# Second pass: reservoir sampling
sampled = []
with open('accelerator_samples_old.txt', 'r') as f:
    for i, line in enumerate(f):
        if i < sample_size:
            sampled.append(line)
        else:
            r = random.randint(0, i)
            if r < sample_size:
                sampled[r] = line

# Write
with open('sampled1.txt', 'w') as f:
    f.writelines(sampled)

print(f"Sampled {len(sampled)} out of {total_lines} lines")