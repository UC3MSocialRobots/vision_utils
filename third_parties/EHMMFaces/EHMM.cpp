#include "EHMM.h"
#include <stdio.h>

//*****************************************************************************

EHMM::EHMM( )
{
    _ehmm = 0;
    _vecSize = 0;
	_trained = false;
}

//*****************************************************************************

EHMM::~EHMM( )
{
	Release( );
}

//*****************************************************************************

void EHMM::Release( )
{
    if ( _ehmm ) 
	{
		cvRelease2DHMM( &_ehmm );  
	}

    _vecSize = 0;
	_trained = false;
}

//*****************************************************************************

void EHMM::Create( int *noStates, int *noMix, int vecSize )
{
	assert( _ehmm == 0 );

    _ehmm = cvCreate2DHMM( noStates, noMix, vecSize ); 

	assert( _ehmm != 0 );
    
    _vecSize = vecSize;   

	_trained = false;
}

//*****************************************************************************

void EHMM::Load( const std::string &fileName )
{
	const int MAX_STATES = 128;

    FILE *file;
    int states[ MAX_STATES ];
    int gaussMix[ MAX_STATES ];
    char tmpChar[ MAX_STATES ];
    int i, k;

    assert( _ehmm == 0 );

    file = fopen( fileName.c_str( ), "rt" );

	assert( file != 0 );

    //Read topology
    fscanf( file, "%s %d\n", tmpChar, states );

    fscanf( file, "%s ", tmpChar ); 

    for( i = 0; i < states[ 0 ]; i++ )
    {
        fscanf( file, "%d ", states + i + 1 );
    }
    
	fscanf( file, "\n" );
    
    //Compute total number of internal states
    int total_states = 0;

    for( i = 0; i < states[ 0 ]; i++ )
    {
        total_states += states[ i+1 ];
    }
    
    //Read number of mixtures
    fscanf( file, "%s ", tmpChar );

    for( i = 0; i < total_states; i++ )
    {
        fscanf( file, "%d ", &gaussMix[ i ] );
    }

    fscanf( file, "\n" );

    fscanf( file, "%s %d\n", tmpChar, &_vecSize );

    _ehmm = cvCreate2DHMM( states, gaussMix, _vecSize );

     //Create HMM with known parameters
	assert( _ehmm != 0 );

    //Consequently read all hmms
    
	CvEHMM* hmm = _ehmm;

    for( i = 0; i < states[ 0 ]+1; i++ )
    {
        fscanf( file, "%s\n", tmpChar ); 
			//printf ("1 Aqui estamos %d\n",file);
			//printf ("2 Aqui estamos %s\n",tmpChar);

		int temp_int;
    
		fscanf( file, "%s %d\n", tmpChar , &temp_int );  
		//printf ("3 Aqui estamos %d\n",file);
		//printf ("4 Aqui estamos %s\n",tmpChar);
		//printf ("5 Aqui estamos %d\n",temp_int );

		assert( temp_int == states[ i ] ); 

        if ( i!= 0 )
        {
            for ( int j = 0; j < states[ i ]; j++)
            {
                CvEHMMState* state = &( hmm->u.state[ j ] );

                fscanf( file, "%s %d\n", tmpChar, &temp_int ); assert( temp_int == j );

                fscanf( file, "%s %d\n", tmpChar, &temp_int ); assert( temp_int == state->num_mix );

                float* mu = state->mu;

				float* inv_var = state->inv_var;

                for( int m = 0; m < state->num_mix; m++)
                {
                    int temp_int;
                    
					fscanf( file, "%s %d %s %f\n", tmpChar, &temp_int, tmpChar, &( state->weight[ m ]) );
                    
					assert( temp_int == m );
                    
					fscanf( file, "%s\n", tmpChar );

                    for ( k = 0; k < _vecSize; k++)
                    {  
                        fscanf( file, "%f ", mu ); 
                        mu++;
                    }            
                    
                    fscanf( file, "\n" );
                    
					fscanf( file, "%s\n", tmpChar );
                    
                    for ( k = 0; k < _vecSize; k++)
                    {
                        fscanf( file, "%f ", inv_var );
                        inv_var++;
                    }
                    
					fscanf( file, "\n" );

                    fscanf( file, "%s %f\n", tmpChar, &( state->log_var_val[ m ]) );                    
                }
            }
        }

        //Read the transition probability matrix
        
		fscanf( file, "%s\n", tmpChar ); 

        float* prob = hmm->transP;

        for ( int j = 0; j < hmm->num_states; j++)
        {
            for ( int k = 0; k < hmm->num_states; k++)
            {
                fscanf( file, "%f ", prob );
                prob++;
            }            
            fscanf( file, "\n" );
        }

        fscanf( file, "%s\n", tmpChar );

        hmm = &( _ehmm->u.ehmm[ i ] );
    }
 
	fclose( file ); 

	_trained = true;
}

//*****************************************************************************

void EHMM::Save( const std::string &fileName )
{
	FILE* file;
	int i;
	int j;
	int m;
	int k;

    assert( _ehmm != 0 );

    file = fopen( fileName.c_str( ), "wt" );

    assert( file != 0 );

    //Write topology
    
	fprintf( file, "%s %d\n", "<NumSuperStates>", _ehmm->num_states );

	fprintf( file, "%s ", "<NumStates>" );

    for( i = 0; i < _ehmm->num_states; i++ )
    {
        fprintf( file, "%d ", _ehmm->u.ehmm[ i ].num_states );
    }

    fprintf( file, "\n" );

    fprintf( file, "%s ", "<NumMixtures>" );
    
	for( i = 0; i < _ehmm->num_states; i++ )
    {
        CvEHMM* ehmm = &( _ehmm->u.ehmm[ i ] );

        for( int j = 0; j < ehmm->num_states; j++ )
        {
            fprintf( file, "%d ", ehmm->u.state[ j ].num_mix );
        }
    }
    fprintf( file, "\n" );

    fprintf( file, "%s %d\n", "<VecSize>", _vecSize );

    //Consequently write all hmms
    
	CvEHMM* hmm = _ehmm;
    
	for( i = 0; i < _ehmm->num_states + 1; i++ )
    {
        if ( hmm->level == 0 )
		{
            fprintf( file, "%s\n", "<BeginEmbeddedHMM>" );
		}
        else
		{
            fprintf( file, "%s\n", "<BeginExternalHMM>" );
		}

        fprintf( file, "%s %d\n", "<NumStates>", hmm->num_states );

        if ( hmm->level == 0 )
        {
            for ( j = 0; j < hmm->num_states; j++)
            {
                CvEHMMState* state = &( hmm->u.state[ j ] );

                fprintf( file, "%s %d\n", "<State>", j );

                fprintf( file, "%s %d\n", "<NumMixes>", state->num_mix );

                float* mu = state->mu;
                float* inv_var = state->inv_var;

                for( m = 0; m < state->num_mix; m++)
                {
                    fprintf( file, "%s %d %s %f\n", "<Mixture>", m, "<Weight>", state->weight[ m ] );

                    fprintf( file, "%s\n", "<Mean>" );

                    for ( k = 0; k < _vecSize; k++)
                    {  
                        fprintf( file, "%f ", mu[ 0 ] ); 
                        mu++;
                    }            
                    
                    fprintf( file, "\n" );
                    
					fprintf( file, "%s\n", "<Inverted_Deviation>" );
                    
                    for ( k = 0; k < _vecSize; k++)
                    {
                        fprintf( file, "%f ", inv_var[ 0 ] );
                        inv_var++;
                    }

                    fprintf( file, "\n" );

                    fprintf( file, "%s %f\n", "<LogVarVal>", state->log_var_val[ m ] );                    
                }
            }
        }

        //Write the transition probability matrix
        fprintf( file, "%s\n", "<TransP>" ); 
        float* prob = hmm->transP;

        for ( int j = 0; j < hmm->num_states; j++)
        {
            for ( int k = 0; k < hmm->num_states; k++)
            {
                fprintf( file, "%f ", *prob );
                prob++;
            }            
            fprintf( file, "\n" );
        }

        if( hmm->level == 0 )
		{
            fprintf( file, "%s\n", "<EndEmbeddedHMM>" );
		}
        else
		{
            fprintf( file, "%s\n", "<EndExternalHMM>" );
		}

        hmm = &( _ehmm->u.ehmm[ i ] );
    }            

    fclose( file );
}

//*****************************************************************************

bool EHMM::GetTrained( )
{
	assert( _ehmm != 0 );

	return _trained;
}

//*****************************************************************************

void EHMM::SetTrained( bool trained )
{
	assert( _ehmm != 0 );

	_trained = trained;
}

//*****************************************************************************

/*
*	Fill in the noStates vector with the number of states
*/
void EHMM::GetNoStates( int *noStates ) const
{
	int i;

	assert( _ehmm != 0 );

	noStates[ 0 ] = _ehmm->num_states;

    for( i = 0; i < _ehmm->num_states; i++ )
    {
       noStates[ i + 1 ] = _ehmm->u.ehmm[ i ].num_states;
    }
}

//*****************************************************************************

/*
*	Fill in the noMix vector with the number of mixtures per state
*/
void EHMM::GetNoMix( int *noMix ) const
{
	int i;
	int j;
	int c;

	assert( _ehmm != 0 );
	
	for( c = 0, i = 0; i < _ehmm->num_states; i++ )
    {
        CvEHMM* ehmm = &( _ehmm->u.ehmm[ i ] );

        for( j = 0; j < ehmm->num_states; j++ )
        {
            noMix[ c++ ] = ehmm->u.state[ j ].num_mix;
        }
	}
}

//*****************************************************************************

int EHMM::GetVecSize( ) const
{
	assert( _ehmm != 0 );

	return _vecSize;
}

//*****************************************************************************

CvEHMM* EHMM::GetCvEHMM( )
{
	assert( _ehmm != 0 );

	return _ehmm;
}

//*****************************************************************************
