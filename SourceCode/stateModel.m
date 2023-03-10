function stateNext = stateModel(state,T)
    F = [1  T 0  0; 
         0  1 0  0;
         0  0 1  T;
         0  0 0  1];
    stateNext = F*state;
end