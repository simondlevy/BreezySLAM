classdef Randomizer
    %Randomizer Internal class for random-number generation
    %   Matlab has its own random-number generators, but this class
    %   provides consistency with our Python and C++ SLAM implementations.
    
    properties (Access = 'private')
        c_randomizer
    end
    
    methods
        
        function r = Randomizer(seed)
            r.c_randomizer = mex_coreslam('Randomizer_init', seed);
        end
        
    end
    
end

