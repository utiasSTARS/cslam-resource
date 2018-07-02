function score = maxprob_metric(MSP, g)
    assert(size(g.edges,2) == 6); % check the passed in graph follows the latest convention
    assert(size(MSP.g_init.edges,2) == 6);
    score = sum(g.edges(:,5)) - sum(MSP.g_init.edges(:,5));
end