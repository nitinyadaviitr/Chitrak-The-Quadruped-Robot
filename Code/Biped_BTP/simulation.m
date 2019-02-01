function simulation( P0, P1, P2, P4, P5, origin, x, z)

 line2 = line([P0(1) P1(1)],[P0(2) P1(2)] , 'LineWidth',3 ,'Color','red');
 line3 = line([P1(1) P2(1)],[P1(2) P2(2)], 'LineWidth',2,'Color','red');
 line4 = line([P0(1) P4(1)],[P0(2) P4(2)] , 'LineWidth',3 ,'Color','green');
 line5 = line([P4(1) P5(1)],[P4(2) P5(2)], 'LineWidth',2,'Color','green');
 line6 = line([origin(1) (origin(1) + x)],[origin(2) z] , 'LineWidth',1 ,'Color','blue','LineStyle', '--');
 ylabel('y'); 
 xlabel('x') ;
 Traj_identi = viscircles(P0,0.001, 'color', 'blue');
%  Traj_identi1 = viscircles(P1,0.001);
 Traj_identi2= viscircles(P2,0.001);
%  Traj_identi3 = viscircles(P4,0.001);
 Traj_identi4= viscircles(P5,0.001, 'Color','green');
 pause(0.1);
 %remove previous identifiers
 delete(line2);
 delete(line3);
 delete(line4);
 delete(line5);
 delete(line6);

end

