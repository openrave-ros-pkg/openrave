// -*- coding: utf-8 -*-
// Copyright (C) 2006-2011 Rosen Diankov <rosen.diankov@gmail.com>
//
// This file is part of OpenRAVE.
// OpenRAVE is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
#ifndef OPENRAVE_IRP6KINEMATIC
#define OPENRAVE_IRP6KINEMATIC

#include <openrave/planningutils.h>
#include <cstdlib>


#include <sstream>

/// manages all connections. At the moment, server can handle only one connectino at a time
class Irp6Kinematic : public ModuleBase
{
public:
    Irp6Kinematic(EnvironmentBasePtr penv) : ModuleBase(penv)
    {
		__description=":Interface Author: Rosen Diankov\n\nSimple text-based server using sockets.";
		RegisterCommand("ikSolvePost",boost::bind(&Irp6Kinematic::ikSolvePost, this,_1,_2),"solves ik for postument");
        //RegisterCommand("fkSolvePost",boost::bind(&Irp6Kinematic::fkSolvePost, this,_1,_2),"solves fk for postument");
        
		a2 = 0.455;
		a3 = 0.67;
		d5 = 0.19;
		d6 = 0.25;
    }

    virtual ~Irp6Kinematic()
    {
        Destroy();
    }

    virtual int main(const std::string& cmd)
    {
        return 0;
    }

    virtual void Destroy()
    {
        Reset();
    }

    virtual void Reset()
    {

    }



protected:
	bool ikSolvePost(ostream& sout, istream& sinput)
	{
		double z_offset_const=0.95;
		
		std::vector<double> local_desired_joints(6);

		// Stale
		const double EPS = 1e-10;

		// Zmienne pomocnicze.
		double Nx, Ox, Ax, Px;
		double Ny, Oy, Ay, Py;
		double Nz, Oz, Az, Pz;
		double s0, c0, s1, c1, s3, c3, s4, c4;
		bool osobliwosc =false;
		double E, F, K, ro, G, H;
		double t5, t_ok;

		// Wczytanie zadanej pozycji z strumienia wejsciowego	
		sinput >>Nx;
		sinput >>Ox;
		sinput >>Ax;
		sinput >>Ny;
		sinput >>Oy;
		sinput >>Ay;
		sinput >>Nz;
		sinput >>Oz;
		sinput >>Az;
		sinput >> Px;
		sinput >> Py;
		sinput >> Pz;
		Pz -= z_offset_const;
		Pz-= d6*Az;
		Py-= d6*Ay;
		Px-= d6*Ax;
		
		//std::cout << Nx << "  " << Ox << "  " << Ax << "  " << Px << "\n"; 
		//std::cout << Ny << "  " << Oy << "  " << Ay << "  " << Py << "\n"; 
		//std::cout << Nz << "  " << Oz << "  " << Az << "  " << Pz << "\n"; 

		//  Wyliczenie Theta1.
		(local_desired_joints)[0] = (atan2(Py, Px));
		s0 = sin((double) (local_desired_joints)[0]);
		c0 = cos((double) (local_desired_joints)[0]);

		// Wyliczenie sin Theta5.
		c4 = Ay * c0 - Ax * s0;
			// Sprawdzenie bledow numerycznych.
		if (fabs(c4 * c4 - 1) > EPS)
		s4 = sqrt(1 - c4 * c4);
		else
		s4 = 0;

		// Wyliczenie Theta4 i Theta6.
		if (fabs(s4) < EPS)
		{
			printf("Osobliwosc p\n");	//Jeden z stawow ustawiam jaka stala wartosc i do niej dobieram druga 
			osobliwosc = true;
			// W przypadku osobliwosci katowi theta4 przypisywana wartosc poprzednia.
			(local_desired_joints)[3] = 0.4;
			t5 = atan2(c0 * Nx + s0 * Ny, c0 * Ox + s0 * Oy);
			(local_desired_joints)[5] = t5-(local_desired_joints)[3];
		}
		else 
		{
			t5 = atan2(-s0 * Ox + c0 * Oy, s0 * Nx - c0 * Ny);
			(local_desired_joints)[5] = t5;

			t_ok = atan2(c0 * Ax + s0 * Ay, Az);

			if (fabs((double)(t_ok )) > fabs((double)(t_ok - M_PI ))) t_ok = t_ok - M_PI;
			if (fabs((double)(t_ok )) > fabs((double)(t_ok + M_PI )))
			  t_ok = t_ok + M_PI;
			(local_desired_joints)[3] = t_ok;
		}  //: else

		// Wyliczenie Theta2.
		c3 = cos((double)(local_desired_joints)[3]);
		s3 = sin((double)(local_desired_joints)[3]);

		E = c0 * Px + s0 * Py - c3 * d5;
		F = -Pz - s3 * d5;
		G = 2 * E * a2;
		H = 2 * F * a2;
		K = E * E + F * F + a2 * a2 - a3 * a3;
		ro = sqrt(G * G + H * H);

		(local_desired_joints)[1] = atan2(K / ro, sqrt(1 - ((K * K) / (ro * ro))))
		  - atan2(G, H);

		// Wyliczenie Theta3.
		s1 = sin((double)(local_desired_joints)[1]);
		c1 = cos((double)(local_desired_joints)[1]);
		(local_desired_joints)[2] = atan2(F - a2 * s1, E - a2 * c1);

		//Wyznaczenie do konca Theta5
		if(fabs(c3) > EPS) s4=Az/c3;
		else s4=(c1*Ax+s1*Ay)/s3;

		(local_desired_joints)[4] = atan2(s4, c4);

		// poprawka w celu dostosowania do konwencji DH
		(local_desired_joints)[2] -= (local_desired_joints)[1] + M_PI_2;
		(local_desired_joints)[3] -= (local_desired_joints)[2]
		  + (local_desired_joints)[1] + M_PI_2;
	
	
		//Korekty do Theta 3 i Theta 6
		if(local_desired_joints[4]<-0.5) local_desired_joints[4]+=2*M_PI;
	
		double c5 = cos(local_desired_joints[5]);
		double s5 = sin(local_desired_joints[5]);
		
		if(fabs(c3*c4*c5-s3*s5-Nz) > EPS && !osobliwosc) local_desired_joints[5]-=M_PI;
		
		//Wypisanie wyniku do strumienia
		sout << local_desired_joints[0];
		sout << " ";
		sout << local_desired_joints[1];
		sout << " ";
		sout << local_desired_joints[2];
		sout << " ";
		sout << local_desired_joints[3];
		sout << " ";
		sout << local_desired_joints[4];
		sout << " ";
		sout << local_desired_joints[5];
		return true;	
	}

	private:

	double a2;
	double a3;
	double d5;
	double d6;
};

#ifdef RAVE_REGISTER_BOOST
#include BOOST_TYPEOF_INCREMENT_REGISTRATION_GROUP()

BOOST_TYPEOF_REGISTER_TYPE(SimpleTextServer::Socket)
BOOST_TYPEOF_REGISTER_TYPE(SimpleTextServer::WORKERSTRUCT)

#endif

#endif
