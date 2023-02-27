function v = vlinspace(init_vector, target_vector, n)

for tv = 1:3
    [v(:, tv)] = linspace(init_vector(tv), target_vector(tv), n);
end
end