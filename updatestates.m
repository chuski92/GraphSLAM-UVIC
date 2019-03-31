function states = updatestates(states,dx)

for i= 1:1:numel(states)
    
    states{i}.value = states{i}.value + dx(states{i}.range);
    
    if strcmp(states{i}.type,'pose')
        states{i}.value=pi2pi(states{i}.value);
    end
    
end