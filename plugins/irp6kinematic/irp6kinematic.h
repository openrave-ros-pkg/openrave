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
    Irp6Kinematic(EnvironmentBasePtr penv) : ModuleBase(penv),EPS(1e-10)
    {
		__description=":Interface Author: Rosen Diankov\n\nSimple text-based server using sockets.";
		RegisterCommand("solveIKPost",boost::bind(&Irp6Kinematic::solveIKPost, this,_1,_2),"solves ik for postument");
        //RegisterCommand("fkSolvePost",boost::bind(&Irp6Kinematic::fkSolvePost, this,_1,_2),"solves fk for postument");
        
		a2 = 0.455;
		a3 = 0.67;
		d5 = 0.19;
		d6 = 0.25;
		z_offset_const=0.95;
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
	bool solveIKPost(ostream& sout, istream& sinput)
	{
		
		std::vector<double> dstJoints(6);

		// Wczytanie zadanej pozycji z strumienia wejsciowego	
		std::vector<double> transMatrix;
		for(int i=0; i<12;i++)
		{
			double tmp;
			sinput >> tmp;
			transMatrix.push_back(tmp);
		}
		
		bool foundSolution = solveIKIrp6(transMatrix,dstJoints);
		
		//Wypisanie wyniku do strumienia
		sout << dstJoints[0];
		sout << " ";
		sout << dstJoints[1];
		sout << " ";
		sout << dstJoints[2];
		sout << " ";
		sout << dstJoints[3];
		sout << " ";
		sout << dstJoints[4];
		sout << " ";
		sout << dstJoints[5];
		return foundSolution;	
	}
	
	bool solveIKIrp6( std::vector<double> transMatrix, std::vector<double>& dstJoints)
	{

		/*	tm(0)	tm(1)	tm(2)	tm(9)
		*	tm(3)	tm(4)	tm(5)	tm(10)
		*	tm(6)	tm(7)	tm(8)	tm(11)
		*
		*				|
		*				V
		*
		*	Nx		Ox		Ax		Px 
		*	Ny		Oy		Ay		Py  
		*	Nz		Oz		Az		Pz
		*/

		// Zmienne pomocnicze.
		double Nx,  Ox,  Ax, Px;
		double Ny,  Oy,  Ay, Py;
		double Nz,/*Oz*/ Az, Pz;	//Oz unused
		double s0, c0, s1, c1, s3, c3, s4, c4;
		bool osobliwosc =false;
		double E, F, K, ro, G, H;
		double t5, t_ok;
	
		if(transMatrix.size()!=12) return false;
		
		Nx=transMatrix[0];
		Ox=transMatrix[1];
		Ax=transMatrix[2];
		Ny=transMatrix[3];
		Oy=transMatrix[4];
		Ay=transMatrix[5];
		Nz=transMatrix[6];
		//Oz=transMatrix[7];
		Az=transMatrix[8];
		Px=transMatrix[9];
		Py=transMatrix[10];
		Pz=transMatrix[11];

		Pz -= z_offset_const;
		Pz-= d6*Az;
		Py-= d6*Ay;
		Px-= d6*Ax;

		//  Wyliczenie Theta1.
		(dstJoints)[0] = (atan2(Py, Px));
		s0 = sin((double) (dstJoints)[0]);
		c0 = cos((double) (dstJoints)[0]);

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
			(dstJoints)[3] = 0.4;
			t5 = atan2(c0 * Nx + s0 * Ny, c0 * Ox + s0 * Oy);
			(dstJoints)[5] = t5-(dstJoints)[3];
		}
		else 
		{
			t5 = atan2(-s0 * Ox + c0 * Oy, s0 * Nx - c0 * Ny);
			(dstJoints)[5] = t5;

			t_ok = atan2(c0 * Ax + s0 * Ay, Az);

			if (fabs((double)(t_ok )) > fabs((double)(t_ok - M_PI ))) t_ok = t_ok - M_PI;
			if (fabs((double)(t_ok )) > fabs((double)(t_ok + M_PI )))
			  t_ok = t_ok + M_PI;
			(dstJoints)[3] = t_ok;
		}  //: else

		// Wyliczenie Theta2.
		c3 = cos((double)(dstJoints)[3]);
		s3 = sin((double)(dstJoints)[3]);

		E = c0 * Px + s0 * Py - c3 * d5;
		F = -Pz - s3 * d5;
		G = 2 * E * a2;
		H = 2 * F * a2;
		K = E * E + F * F + a2 * a2 - a3 * a3;
		ro = sqrt(G * G + H * H);

		(dstJoints)[1] = atan2(K / ro, sqrt(1 - ((K * K) / (ro * ro))))
		  - atan2(G, H);

		// Wyliczenie Theta3.
		s1 = sin((double)(dstJoints)[1]);
		c1 = cos((double)(dstJoints)[1]);
		(dstJoints)[2] = atan2(F - a2 * s1, E - a2 * c1);

		//Wyznaczenie do konca Theta5
		if(fabs(c3) > EPS) s4=Az/c3;
		else s4=(c1*Ax+s1*Ay)/s3;

		(dstJoints)[4] = atan2(s4, c4);

		// poprawka w celu dostosowania do konwencji DH
		(dstJoints)[2] -= (dstJoints)[1] + M_PI_2;
		(dstJoints)[3] -= (dstJoints)[2]
		  + (dstJoints)[1] + M_PI_2;
	
	
		//Korekty do Theta 3 i Theta 6
		if(dstJoints[4]<-0.5) dstJoints[4]+=2*M_PI;
	
		double c5 = cos(dstJoints[5]);
		double s5 = sin(dstJoints[5]);
		
		if(fabs(c3*c4*c5-s3*s5-Nz) > EPS && !osobliwosc) dstJoints[5]-=M_PI;	
		
		return true;
	}

	private:

	const double EPS;

	double a2;
	double a3;
	double d5;
	double d6;
	double z_offset_const;
};

#ifdef RAVE_REGISTER_BOOST
#include BOOST_TYPEOF_INCREMENT_REGISTRATION_GROUP()

BOOST_TYPEOF_REGISTER_TYPE(SimpleTextServer::Socket)
BOOST_TYPEOF_REGISTER_TYPE(SimpleTextServer::WORKERSTRUCT)

#endif

#endif
