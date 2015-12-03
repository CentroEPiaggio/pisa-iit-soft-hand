function [bm_hex,bm_bin,bm,collMat] = gen_bitmask()
% function [bm_hex,bm_bin,bm,collMat] = gen_bitmask()
% 
% Compute bitmasks for the SoftHand links (since Gazebo6)
% All links should collide with all the others, but:
% - palm link should not collide with knuckles
% - each finger link should not collide with previous/next ones
% 
% outputs:
% - bm_hex:     hexadecimal bitmask (to be used in the sdf)
% - bm_binary:  binary bitmask, just for checking
% - bm:         decimal bitmask, just for checking
% - collMat:    associated collision matrix, just for checking

% total number of finger links (with collisions)
fingerLinks = 19;
% index of first link of each finger (1 is the palm, then thumb, index,...)
fingerStart = 1+[1 4 8 12 16];

bm = zeros(fingerLinks+1,1);

% start with the palm
bm(1) = 1;
% number of bits (2 is the number to start)
k = 2;

% add each finger link
for i=2:fingerLinks+1
    
    start_j = i-2;
    end_j = 1;
    % case of the first link of each finger
    if(any(fingerStart == i))
        bm(i) = 2^k-1 - bm(1);
        start_j = i-1;
        end_j = 2;
    % case of any other link
    else
        bm(i) = 2^k-1 - bm(i-1);
    end

    %%
%     should_run_again = true;

%     while(should_run_again)
        % % index for which it is not possible to update the most
        % % significan bit because of close collisions induced
%         badj = i;
%         should_run_again = false;
    %%
        any_done = false;
        for j=start_j:-1:end_j
            if(bitand(bm(i),bm(j),'uint32') == 0)
                %%
%                 if(badj - j < 2)
%                     should_run_again = true;
%                     disp(['should_run_again i:' num2str(i) ', j:' ...
%                         num2str(j) ', badj:' num2str(badj)]);
%                     continue;
%                 else
%                     badj = j;
                %%
                    any_done = true;
                    bm(j) = bm(j) + 2^k;
                %%
%                 end
                %%
            end
        end
%         if(or(any_done,should_run_again))
        if(any_done)
            bm(i) = bm(i) + 2^k;
            k = k+1;
        end
%     end
end

disp(['Number of bits needed: ' num2str(k)]);

% convert to hexadecimal and binary
for i=1:fingerLinks+1
    bm_hex(i,:) = ['''0x' dec2hex(bm(i),ceil(k/4)) ''',']; %#ok
    bm_bin(i,:) = ['''0b' dec2bin(bm(i),k) ''',']; %#ok
end

% compute collision matrix
collMat = zeros(fingerLinks+1);
for i=1:fingerLinks+1
    collMat(i,i) = 2;
    for j=i+1:fingerLinks+1
        if(bitand(bm(i),bm(j),'uint32') ~= 0)
            collMat(i,j) = 1;
            collMat(j,i) = 1;
        end
    end
end
% disp(collMat);

end

    