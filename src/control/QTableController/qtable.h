#ifndef _QTABLE_H_
#define _QTABLE_H_

#include <vector>

class QTable {
public:
    QTable(unsigned int _xdim, unsigned int _udim);
    ~QTable();

    bool    load( const char* qtable_definition_fname );
    bool    save( const char* qtable_definition_fname , bool format_ascii = true );

    void    set_value(const double* x , const double* u, double q);
    void    set_value(unsigned long int id_x , unsigned int id_u, double q);

    double  get_value(const double* x , const double* u);
    double  get_value(const double* x , unsigned int id_u);
    double  get_value(unsigned long int id_x , unsigned int id_u);

    unsigned long int   get_state_id(const double* x);
    unsigned int        get_action_id(const double* u);
    void                copy_action( unsigned int id_u , double* u_target );

    const std::vector< double* > action_list();
    const double*  action(unsigned int i);
    unsigned int   number_of_actions();

protected:
    unsigned int xdim;
    unsigned int udim;

    double** q_array;;
    unsigned int long q_array_size;

    std::vector< double > state_supports_min;
    std::vector< double > state_supports_max;
    std::vector< double > state_supports_number;
    std::vector< double* > actions;
    std::vector<unsigned long int> state_mult;

    bool create_array();
    bool init_array(double _init_value);
    bool destroy_array();
};

#endif
