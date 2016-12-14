function drawSphere(position,diameter)

		diameter = 0.1;
		[X,Y,Z] = sphere;
		X=X*diameter;
		Y=Y*diameter;
		Z=Z*diameter;
		X=X+position(1);
		Y=Y+position(2);
		Z=Z+position(3);
		surf(X,Y,Z);
		%~ shading flat

	end