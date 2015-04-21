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
#include <limits>


#include <sstream>

/// manages all connections. At the moment, server can handle only one connectino at a time
class Irp6Kinematic : public ModuleBase
{
public:
    Irp6Kinematic(EnvironmentBasePtr penv) : ModuleBase(penv),EPS(1e-10)
    {
		__description=":Interface Author: Rosen Diankov\n\nSimple text-based server using sockets.";
		RegisterCommand("solveIKPost",boost::bind(&Irp6Kinematic::solveIKPost, this,_1,_2),"solves ik for postument");
		RegisterCommand("solveIKTrack",boost::bind(&Irp6Kinematic::solveIKTrack, this,_1,_2),"solves ik for track");
        RegisterCommand("solveFKPost",boost::bind(&Irp6Kinematic::solveFKPost, this,_1,_2),"solves fk for postument");
        RegisterCommand("solveFKTrack",boost::bind(&Irp6Kinematic::solveFKTrack, this,_1,_2),"solves fk for track");
        
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
	bool solveIKTrack(ostream& sout, istream& sinput)
	{
		std::vector<double> dstJoints(7);
		std::vector<double> manipDstJoints(6); // rozwiazanie manipulatora bez tor jezdnego

		// Wczytanie zadanej pozycji z strumienia wejsciowego	
		std::vector<double> transMatrix;
		for(int i=0; i<12;i++)
		{
			double tmp;
			sinput >> tmp;
			transMatrix.push_back(tmp);
		}
		double Py=transMatrix[10];
		bool foundSolution=false;
		
		for(double j0=0;j0<1.2;j0+=0.05)
		{
			transMatrix[10]=Py-j0;
			foundSolution = solveIKIrp6(transMatrix,manipDstJoints);
			if(foundSolution)
			{
				dstJoints[0]=j0;
				dstJoints[1]=manipDstJoints[0];
				dstJoints[2]=manipDstJoints[1];
				dstJoints[3]=manipDstJoints[2];
				dstJoints[4]=manipDstJoints[3];
				dstJoints[5]=manipDstJoints[4];
				dstJoints[6]=manipDstJoints[5];
				break;
			}
		}
		
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
		sout << " ";
		sout << dstJoints[6];
		return foundSolution;			
	}

	bool solveFKTrack(ostream& sout, istream& sinput)
	{
		std::vector<double> transMatrix;

		// Wczytanie zadanej pozycji z strumienia wejsciowego	
		std::vector<double> joints;
		double track;
		
		sinput >> track;
		for(int i=0; i<6;i++)
		{
			double tmp;
			sinput >> tmp;
			joints.push_back(tmp);
		}
		
		bool foundSolution = solveFKIrp6(joints,transMatrix);
		transMatrix[10]+=track;
		
		//Wypisanie wyniku do strumienia
		sout << transMatrix[0];
		sout << " ";
		sout << transMatrix[1];
		sout << " ";
		sout << transMatrix[2];
		sout << " ";
		sout << transMatrix[3];
		sout << " ";
		sout << transMatrix[4];
		sout << " ";
		sout << transMatrix[5];
		sout << " ";
		sout << transMatrix[6];
		sout << " ";
		sout << transMatrix[7];
		sout << " ";
		sout << transMatrix[8];
		sout << " ";
		sout << transMatrix[9];
		sout << " ";
		sout << transMatrix[10];
		sout << " ";		
		sout << transMatrix[11];

		return foundSolution;	
	}


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
	
	bool solveFKPost(ostream& sout, istream& sinput)
	{
		std::vector<double> transMatrix;

		// Wczytanie zadanej pozycji z strumienia wejsciowego	
		std::vector<double> joints;
		for(int i=0; i<6;i++)
		{
			double tmp;
			sinput >> tmp;
			joints.push_back(tmp);
		}
		
		bool foundSolution = solveFKIrp6(joints,transMatrix);
		
		//Wypisanie wyniku do strumienia
		sout << transMatrix[0];
		sout << " ";
		sout << transMatrix[1];
		sout << " ";
		sout << transMatrix[2];
		sout << " ";
		sout << transMatrix[3];
		sout << " ";
		sout << transMatrix[4];
		sout << " ";
		sout << transMatrix[5];
		sout << " ";
		sout << transMatrix[6];
		sout << " ";
		sout << transMatrix[7];
		sout << " ";
		sout << transMatrix[8];
		sout << " ";
		sout << transMatrix[9];
		sout << " ";
		sout << transMatrix[10];
		sout << " ";		
		sout << transMatrix[11];

		return foundSolution;	
	}
	
	bool solveFKIrp6( std::vector<double> jointValues, std::vector<double>& transMatrix)
	{
	
	transMatrix.clear();
	transMatrix = std::vector<double>(12);
		
	// poprawka w celu uwzglednienia konwencji DH

	jointValues[2] += jointValues[1] + M_PI_2;
	jointValues[3] += jointValues[2];

	// Parametry pomocnicze - przeliczenie zmiennych.
	const double s1 = sin(static_cast<double>(jointValues[0]));
	const double c1 = cos(static_cast<double>(jointValues[0]));
	const double s2 = sin(static_cast<double>(jointValues[1]));
	const double c2 = cos(static_cast<double>(jointValues[1]));
	const double s3 = sin(static_cast<double>(jointValues[2]));
	const double c3 = cos(static_cast<double>(jointValues[2]));
	const double s4 = sin(static_cast<double>(jointValues[3]));
	const double c4 = cos(static_cast<double>(jointValues[3]));
	const double s5 = sin(static_cast<double>(jointValues[4]));
	const double c5 = cos(static_cast<double>(jointValues[4]));
	const double s6 = sin(static_cast<double>(jointValues[5]));
	const double c6 = cos(static_cast<double>(jointValues[5]));

	// Proste zadanie kinematyki.
	transMatrix[0] =  (c1 *  s4 * c5 + s1 * s5) * c6 + c1 * c4 * s6;	// NX
	transMatrix[1] = -(c1 *  s4 * c5 + s1 * s5) * s6 + c1 * c4 * c6;	// OX
	transMatrix[2] =   c1 *  s4 * s5 - s1 * c5;							// AX
	transMatrix[9] =   c1 * (a2 * c2 + a3 * c3  + d5 * c4);  			// PX
	transMatrix[3] =  (s1 *  s4 * c5 - c1 * s5) * c6 + s1 * c4 * s6;	// NY2
	transMatrix[4] = -(s1 *  s4 * c5 - c1 * s5) * s6 + s1 * c4 * c6;	// OY
	transMatrix[5] =   s1 *  s4 * s5 + c1 * c5;							// AY
	transMatrix[10] =   s1 * (a2 * c2 + a3 * c3  + d5 * c4);			// PY
	transMatrix[6] =   c4 *  c5 * c6 - s4 * s6;							// NZ
	transMatrix[7] = - c4 *  c5 * s6 - s4 * c6;							// OZ
	transMatrix[8] =   c4 *  s5;										// AZ
	transMatrix[11] = - a2 *  s2 - a3 * s3 - d5  * s4 + z_offset_const;	// PZ
	
	transMatrix[11]+= d6*transMatrix[8]; //Pz-= d6*Az;
	transMatrix[10]+= d6*transMatrix[5]; //Py-= d6*Ay;
	transMatrix[9] += d6*transMatrix[2]; //Px-= d6*Ax;
	
	return true;
	
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
		
		//czy ktorakolwiek wartosc to NaN
		if(isnan(dstJoints[0]) || isnan(dstJoints[1]) || isnan(dstJoints[2]) || isnan(dstJoints[3]) || isnan(dstJoints[4]) || isnan(dstJoints[5])) return false;
		//czy rozwiazanie zawiera sie w limitach
		else if( -2.96705973> 	dstJoints[0]	||	dstJoints[0]	>	2.96705973)		return false; //Joint 1
		else if( -1.8		>	dstJoints[1]	||	dstJoints[1]	>	-0.872664626)	return false; //Joint 2
		else if( -0.3		>	dstJoints[2]	||	dstJoints[2]	>	0.5)			return false; //Joint 3
		else if( -1.57		>	dstJoints[3]	||	dstJoints[3]	>	1.57)			return false; //Joint 4
		else if( -0.5		>	dstJoints[4]	||	dstJoints[4]	>	5.14)			return false; //Joint 5
		else if( -3.0		>	dstJoints[5]	||	dstJoints[5]	>	2.9)			return false; //Joint 6
		else return true;
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
