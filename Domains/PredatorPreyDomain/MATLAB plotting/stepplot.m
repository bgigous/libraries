% Plots things with a starting location marked as a star
clear all
close all

nTrials=5;
epochnum = 0;
figure
epsave = zeros(5,100,60);
maxnumnn = 0; %9;
for nnnum=0:maxnumnn
    for p=0:nTrials-1
        subplot(5,nTrials,(nnnum*nTrials)+p+1)

        fid = sprintf('epoch_%i_n%i_t%i.csv',epochnum,nnnum,p);
        ep = load(fid);
        hold all

        val = ep(end,1);
        ep(end,:)=[];
        
        epsave(p+1,:,:  ) = ep;
        title(val);

        for i=1:2:size(ep,2)
            plot(ep(:,i),ep(:,i+1),'o-')
            plot(ep(1,i),ep(1,i+1),'*')
        end
        hold off
    end
end
%%

% Plots things with a starting location marked as a star
clear all
close all

EP = 9;

for p=0:EP
    figure
        fid = sprintf('epoch_%i_n%i_t%i.csv',p,0,0)
        title(fid)
        
        ep = load(fid);
        hold all

        val = ep(end,1);
        ep(end,:)=[];
        
        epsave(p+1,:,:  ) = ep;
        %title(val);

        for i=1:2:size(ep,2)
            plot(ep(:,i),ep(:,i+1),'o-')
            plot(ep(1,i),ep(1,i+1),'*')
        end
        hold off
end