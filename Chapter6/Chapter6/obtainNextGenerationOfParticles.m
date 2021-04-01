function iNextGeneration = obtainNextGenerationOfParticles(W, nParticles)
    % Selection based on the particle weights
    CDF = cumsum(W)/sum(W);
    iSelect  = rand(nParticles, 1); % Random numbers
    % Indeces of the new particles
    CDFg = [0; CDF];
    indg = [1; (1:nParticles).'];
    iNextGeneration_float = interp1(CDFg, indg, iSelect, 'linear');
    iNextGeneration=round(iNextGeneration_float + 0.5); % Round the indeces
end