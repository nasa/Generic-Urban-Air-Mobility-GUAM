% This file is used to combine multiple trim tables into one table.
% Specifically, the file will combine the trim tables (e.g., level, climb, descent)
% for phases of flight into one table for use with generating the gain scheduled
% controller.
% 
% Written by Mike Acheson 7.5.2023, NASA Langley Research Center (LaRC)
% Dynamic Systems and Control Branch (DSCB), D-316
% Contact: michael.j.acheson@nasa.gov

%**************************************************************************

clear XEQ FreeVar_Table Trans_Table% Clear this variable as dimensions may change 

% ************* Preload the trim files ************************************
f_mat   = {};
fcount  = 0;
trim_len    = length(trim_fnames);
clear succ_ind;
for loop = 1:trim_len
    fname = fullfile(fpath, trim_fnames{loop});
    if ~exist(fname,'file')
        fprintf(1,'The following specified trim file does not exist: %s\n', fname);
        succ_ind(loop) = false;
        continue
    end
    f_mat{loop} = matfile(fullfile(fpath, trim_fnames{loop}));
    succ_ind(loop) = true;
    fcount = fcount +1;
end
fprintf(1, 'Successfully loaded %i of %i trim files.\n', fcount, trim_len);
disp('***********************************************************************');
% if fcount <= 1 
%     fprintf(1, 'Only %i trim files were successfully loaded... Exiting\n',fcount);
%     return
% end

% ******* Verify a common UH schedule for all trim files ************
t_ind = find(succ_ind);
for loop = 1:fcount-1
   if any(f_mat{t_ind(loop)}.UH~=f_mat{t_ind(loop+1)}.UH)
        fprintf(1, 'There is a mismatch between UH in the trim files %i and %i... The UH vectors must be identical... Exiting\n', t_ind(loop), t_ind(loop+1));
        return
   end

   % Verify that transition start and end are common between all the files 
   if f_mat{t_ind(loop)}.trans_start ~= f_mat{t_ind(loop+1)}.trans_start || ...
       f_mat{t_ind(loop)}.trans_end ~= f_mat{t_ind(loop+1)}.trans_end
       fprintf(1, 'There is a mismatch between trans_starts and/or trans_ends in the trim files %i and %i... The transion velocities must be identical... Exiting\n', t_ind(loop), t_ind(loop+1));
       return   
   end
end
fprintf(1,'SUCCESS: Validation that UH vector is identical for all trim files.\n ');

% Validate that UH is monontonically increasing
if any(diff(f_mat{t_ind(1)}.UH)<=0)
    fprintf(1,"The UH vector is not monotonically increasing... Exiting\n");
    return;
end

UH_concat   = f_mat{t_ind(1)}.UH;

% *********** Cycle thru each file and concatenate WH, R and data ***************
WH_concat   = [];
R_concat    = [];
for loop = 1:fcount
    % Concatenate WH and R if not repeated
    for WH_inner = 1:length(f_mat{t_ind(loop)}.WH)
        for R_inner = 1:length(f_mat{t_ind(loop)}.R)
            if loop > 1 && ~isempty(find(f_mat{t_ind(loop)}.WH(1,WH_inner) == WH_concat,1)) && ~isempty(find(f_mat{t_ind(loop)}.R(1,R_inner) == R_concat,1))
                fprintf(1,'Identical UH, WH and RH are repeated in more than one file.  Need to resolve this discrepancy.  Exiting..\n')
                return
            else
                if isempty(find(f_mat{t_ind(loop)}.WH(1,WH_inner) == WH_concat,1))
                    % Just concatenate the current trim file values
                    WH_concat   = [WH_concat f_mat{t_ind(loop)}.WH(1,WH_inner)];
                end
                if isempty(find(f_mat{t_ind(loop)}.R(1,R_inner) == R_concat,1))
                    % Just concatenate the current trim file values
                    R_concat    = [R_concat f_mat{t_ind(loop)}.R(1,R_inner)];
                end
            end % End of if loop > 1...
            
            % Check the status of monotonic increasing for WH_concat and R_concat
            if any(diff(WH_concat)<=0)
                fprintf(1,"The concatenated WH vector is not monotonically increasing... Exiting\n");
                return;
            end

            if any(diff(R_concat)<=0)
                fprintf(1,"The concatenated WH vector is not monotonically increasing... Exiting\n");
                return;
            end

            % Determine indices of WH, and R
            W_ind = find(WH_concat==f_mat{t_ind(loop)}.WH(1,WH_inner),1,'first');
            R_ind = find (R_concat==f_mat{t_ind(loop)}.R(1,R_inner),1,'first');
            disp('')
            XEQ(:,:,W_ind, R_ind) = f_mat{t_ind(loop)}.XEQ';

            % Update the FreeVar_Table
            % th phi p q r flp ail elv rud rl rt pp 

            FreeVar_Table(1:3,:, W_ind, R_ind) = [f_mat{t_ind(loop)}.FreeVar_hover; ...
                f_mat{t_ind(loop)}.FreeVar_trans; f_mat{t_ind(loop)}.FreeVar_cruise]; 
            Trans_Table (1:2,W_ind,R_ind)       = [f_mat{t_ind(loop)}.trans_start; f_mat{t_ind(loop)}.trans_end];

        end % End of for R_inner...
    end % End of for WH_inner...
end

% *************************************************************************

% Output the resulting trim file
temp_fname = fullfile(out_trim_fpath, out_trim_fname);
save(temp_fname,'XEQ','UH_concat','WH_concat','R_concat','FreeVar_Table', 'Trans_Table', '-v7.3');
fprintf(1,'Successfully created concatenated XEQ trim file:\n%s\n Exiting...\n', temp_fname)









