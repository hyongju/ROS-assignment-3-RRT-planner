clear all;close all;clc;

max_iter = 30;

q0 = [0.1 0.1];
q1 = [0.9 0.9];
V1 = q0;
E1 = zeros(max_iter+1,max_iter+1);
V2 = q1;
E2 = zeros(max_iter+1,max_iter+1);
ste_size = 0.2;
%ste_ratio= 0.5;

% obstacle ball with radius 0.1 centered at 0.5;
obs_cen = [0.5 0.5];
obs_rad = 0.1;

for i = 1:max_iter
    exit = 0;
    % do until finding sample in Q_free
    while(~exit)
        q_rand = rand(1,2);
        if norm(q_rand - obs_cen) > obs_rad
            exit = 1;
        end
    end
    % find minimum distance node from tree to q_rand
    [~,idx1] = min_vertex(V1,q_rand);
    % create new node respecting step size ////
    if norm(q_rand - V1(idx1,:)) >= ste_size;
        q_new1 = V1(idx1,:)+(q_rand - V1(idx1,:))/norm(q_rand - V1(idx1,:)) * ste_size;
    else
        q_new1 = q_rand;
    end 
    % check if the new node is collision free and add to the tree if it is
    % collision free
    if ~(norm(q_new1 - obs_cen) <= obs_rad)
        E1(idx1,size(V1,1)+1) = 1;
        V1 = [V1;q_new1];
        if norm(q_new1 - q1)<= ste_size
            E1(size(V1,1),size(V1,1)+1) = 1;
            V1 = [V1;q1];
            break;
        end
    end          

end

figure,
plot(V1(:,1),V1(:,2),'Marker','.','MarkerSize',12,'LineStyle','none'); hold on;
for i=1:size(V1,1)
    for j =1:size(V1,1)
        if E1(i,j) == 1
            line([V1(i,1),V1(j,1)],[V1(i,2),V1(j,2)],'Color','b');
        end
    end
end

t = 0:0.01:2*pi;
plot(obs_rad*cos(t)+obs_cen(1),obs_rad*sin(t)+obs_cen(2),'LineStyle','-','LineWidth',1);hold on;

axis equal;    
axis([0 1 0 1]);

