#include <iostream>
#include <Eigen/Dense>

int main()
{
	int c = 0;


	while (c < 2 || c > 3)
	{

		std::cout << "Interpolation 2D(x,y) or 3D(x,y,z)? (choose 2/3):";
		std::cin >> c;
	}

	if (c == 3)
	{

		double x[4], y[4], z[4], xp, yp;

		x[0] = 1; y[0] = 1; z[0] = 1;
		x[1] = 2; y[1] = 1; z[1] = 2;
		x[2] = 1; y[2] = 2; z[2] = 3;
		x[3] = 2; y[3] = 2; z[3] = 4;

		std::cout << "Give me the coordinates (x,y)\n";
		std::cout << "x:";
		std::cin >> xp;
		std::cout << "y:";
		std::cin >> yp;

		double f, m, m1, m2, m3, m4;
		double x1, x2, y1, y2;

		x1 = x[0];
		x2 = x[1];
		y1 = y[0];
		y2 = y[2];

		m = ((x2 - x1) * (y2 - y1));
		m1 = z[0] * (x2 - xp) * (y2 - yp);
		m2 = z[1] * (xp - x1) * (y2 - yp);
		m3 = z[2] * (x2 - xp) * (yp - y1);
		m4 = z[3] * (xp - x1) * (yp - y1);

		f = m * (m1 + m2 + m3 + m4);
		std::cout << "The solution z is: " << f;
	}
	else if (c == 2)
	{
		char h;

		std::cout << "Lineal or polinomical interpolation?: (l/p):";
		std::cin >> h;


		int points;
		double xp;
		std::cout << "How many points are you going to introduce?:";
		std::cin >> points;
		double pp[points][2], aux[2][2];

		std::cout << "Give me the (x,z) points:\n";
		for (int i = 0; i < points; i++)
		{
			std::cout << "x(" << i + 1 << "):";
			std::cin >> pp[i][0];
			std::cout << "z(" << i + 1 << "):";
			std::cin >> pp[i][1];
		}
		
		std::cout << "Give me the coordinate (x)\n";
		std::cout << "x:";
		std::cin >> xp;
		
		
		if (h == 'p')
		{

			int cont=1;	
			double y=1;
			Eigen::MatrixXd m(points, points);
			Eigen::VectorXd k(points, 1);
			for(int i=0;i<points;i++){
				m(i,0)=1;
				k(i,0) = pp[i][1];
				
				}
				
			for(int i=1;i<points;i++){
				for(int j=0;j<points;j++){
				
					for (int k = 0; k < cont; k++)
					{
						y = y*(pp[j][0]);

					}
					
					m(j,i)=y;
					y=1;
				}
				cont++;
				
				
			
			}
	
			
			Eigen::VectorXd x = m.colPivHouseholderQr().solve(k);
			int cnt = 1;
			double sol=0;
			y=1;

			
			for (int i = 0; i < points; i++)
			{
				if (i == 0)
				{
					sol = sol + x[i];
				}
				else
				{
					for (int j = 0; j < cnt; j++)
					{
						y=y*xp;
					}
					cnt++;
					sol = sol + x[i] * y;
					y = 1;
				}
			}

			std::cout << "The solution z is:\n"
					  << sol << std::endl;
			
	

			
			
			
		}
		else if (h == 'l')
		{

			//interpolacion lineal
			double  x1, x2, z1, z2, f;




			for (int i = 0; i < points; i++)
			{
				if (pp[i][0] >= xp)
				{
					aux[0][0] = pp[i - 1][0];
					aux[1][0] = pp[i][0];
					aux[0][1] = pp[i - 1][1];
					aux[1][1] = pp[i][1];
					break;
				}
			}

			x1 = aux[0][0];
			x2 = aux[1][0];
			z1 = aux[0][1];
			z2 = aux[1][1];

			f = z1 + (z2 - z1) * (xp - x1) / (x2 - x1);

			std::cout << "The solution z is: " << f << std::endl;
		}
	}

	return 0;
}
