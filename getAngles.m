%This function generates the joint angles of the phi and theta
%motors used on the robot for enph 253.
function result = getAngles(x, y, N, xPos, yPos, theta, phi)
    

angles = [0;0];
%Candidates matrix stores possible angle values that will move the arm into
%the appropriate location. The theta value is stored in the first column,
%and the phi value is stored in the second column.
candidates = [];
  for k = 1:N
    for j = 1:N
      if (abs(x - xPos(k,j)) < 0.1) && (abs(y - yPos(k,j)) < 0.1) 
        newRow = [k j];
        candidates = [candidates ; newRow];
      end
    end
  end

  %This block of code may be entirely unnessary - commented out!
  %The 150 is near the centre of the values for the theta joint
  %The 90 is near teh centre of the values for the phi joint
  %for i = 1:length(candidates)
   % if(abs(180*theta(1, candidates(i,1))/pi - 150) < 60 && abs(180*phi(1,candidates(i,2))/pi - 90) < 60)
    angles(1,1) = 180*theta(1,candidates(1,1))/pi;
    angles(2,1) = 180*phi(1,candidates(1,2))/pi;
  %  end
  %end
  result = angles;
end