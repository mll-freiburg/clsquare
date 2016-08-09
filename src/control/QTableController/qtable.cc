#include "qtable.h"
#include <fstream>
#include <sstream>
#include "global.h"
#include <assert.h>

QTable::QTable(unsigned int _xdim, unsigned int _udim)
        : xdim(_xdim), udim(_udim)
{
    q_array = 0;
    q_array_size = 0;
}

QTable::~QTable()
{
    ;
}

const std::vector< double* > QTable::action_list()
{
    return this->actions;
}

const double* QTable::action(unsigned int i)
{
    assert(i>=0 && i<this->actions.size());
    return this->actions[i];
}

unsigned int QTable::number_of_actions()
{
    return this->actions.size();
}

void QTable::set_value(const double* x , const double* u, double q)
{
    assert(this->q_array!=0);
    this->set_value( this->get_state_id(x) , this->get_action_id(u) , q );
}

 void  QTable::set_value(unsigned long int id_x , unsigned int id_u, double q)
 {
     assert(this->q_array!=0);
     assert( id_x >= 0 && id_x < this->q_array_size );
     assert( id_u >= 0 && id_u < this->actions.size() );
     this->q_array[id_x][id_u] = q;
 }

double QTable::get_value(const double* x , const double* u)
{
    assert(this->q_array!=0);
    return this->get_value( this->get_state_id(x) , this->get_action_id(u) );
}

double QTable::get_value(const double* x , unsigned int id_u)
{
    assert(this->q_array!=0);
    return this->get_value( this->get_state_id(x) , id_u );
}

 double  QTable::get_value(unsigned long int id_x , unsigned int id_u)
 {
     assert(this->q_array!=0);
     assert( id_x >= 0 && id_x < this->q_array_size );
     assert( id_u >= 0 && id_u < this->actions.size() );
     return q_array[id_x][id_u];
 }

 void QTable::copy_action( unsigned int id_u , double* u_target )
 {
     assert(this->q_array!=0);
     assert( id_u >= 0 && id_u < this->actions.size() );
     for (unsigned int i=0; i<this->udim; i++) u_target[i]=this->actions[id_u][i];
 }

unsigned int QTable::get_action_id(const double* u)
{
    assert(this->q_array!=0);
    bool found=false;
    unsigned int id = 0;

    for (id=0; id<this->actions.size(); id++) {
        bool equal=true;
        for (unsigned int i=0; i<this->udim; i++) {
            if (u[i] != this->actions[id][i]) { equal=false; break; }
        }
        if (equal) {
            found=true;
            break;
        }
    }

#if 0
    IOUT("Found action id: " << id << " for action: ");
    for (unsigned int i=0; i<this->udim; i++) std::cerr << u[i] << " ";
    std::cerr << "\n";
#endif

    assert(found);

    return id;
}

unsigned long int QTable::get_state_id(const double* x)
{
    assert(this->q_array!=0);
    unsigned int descrete_state[this->xdim];
    for (unsigned int i=0; i<this->xdim; i++) {
        if (x[i] <= this->state_supports_min[i]) { descrete_state[i] = 0; continue; }
        if (x[i] >= this->state_supports_max[i]) { descrete_state[i] = this->state_supports_number[i]-1; continue; }

        double stepsize = (this->state_supports_max[i] - this->state_supports_min[i]) / (this->state_supports_number[i]-1);
        int descrete = (int) (stepsize * (x[i]-this->state_supports_min[i]));
        if (descrete <0) descrete = 0;
        if (descrete > this->state_supports_number[i]-1) descrete=this->state_supports_number[i]-1;
        descrete_state[i] = descrete;
    }

    unsigned long int res = 0;
    for (unsigned int i=0; i<this->xdim; i++) res+=this->state_mult[i]*descrete_state[i];
#if 0
    IOUT("Got state: ");
    for (unsigned int i=0; i<xdim; i++) std::cerr << x[i] << " ";
    IOUT("computed discrete state: ");
    for (unsigned int i=0; i<xdim; i++) std::cerr << descrete_state[i] << " ";
    std::cerr << "\n";
    IOUT("Computed id" << res);
#endif
    return res;
}

bool QTable::load( const char* qtable_definition )
{
    std::ifstream in( qtable_definition );
    if (!in) {
        EOUT("Can not load file : " << qtable_definition << " for reading!");
        return false;
    }

    state_supports_min.clear();
    state_supports_max.clear();
    state_supports_number.clear();
    actions.clear();

    std::string line;
    do {
        getline(in, line);
    } while ( line[0] == '#');

    std::string command;

    { std::stringstream ss(line);
        ss >> command;
        if (command != "dimensions") {EOUT("Error in parsing, dimensions expected"); return false;}
        unsigned int _xdim, _udim;
        ss >> _xdim;
        ss >> _udim;
        if (!ss) {EOUT("Error in parsing dimensions"); return false;}
        if (_xdim!=this->xdim || _udim!=this->udim) {
            EOUT("qtable dimensions not matching plant, expected xdim " << this->xdim << " udim " << this->udim << " got " << _xdim << " " << _udim);
            return false;
        }
    }

    for (unsigned int i=0; i<xdim; i++) {
        getline(in, line);
        std::stringstream ss(line);
        ss >> command;
        if (!in || command != "xsupports") {EOUT("Error in parsing, xsupports expected for dim " << i); return false;}
        int min, max, n;
        ss >> min >> max >> n;
        if (!ss) {EOUT("Error in parsing xspupports in dimension " << i); return false;}
        this->state_supports_max.push_back(max);
        this->state_supports_min.push_back(min);
        this->state_supports_number.push_back(n);
    }

    getline(in, line);

    { std::stringstream ss(line);
        ss >> command;
        if (command != "actions") {EOUT("Error in parsing, actions expected"); return false;}
        char c;
        while (ss) {
            ss >> c;
            if (!ss && this->actions.size()>0) break;
            if (c!='(') { EOUT("Error in parsing action, expected [(] got ["<< (char)c <<"]"); return false;}
            double* a = new double[this->udim];
            for (unsigned int i=0; i<this->udim; i++) {
              ss >> a[i];
            }
            if (!ss) { EOUT("Error in parsing action entry, may be not enough entries in brackets?"); return false;}
            ss >> c;
            if (c!=')') { EOUT("Error in parsing action, expected )"); return false;}
            this->actions.push_back( a );
        }
    }

    if (!this->create_array()) {
        EOUT("Can not create array.");
        return false;
    }

    getline(in, line);

    bool load_data = false;
    bool data_format_ascii = true;

    { std::stringstream ss(line);
        ss >> command;
        if (!(command == "init" || command== "data")) {EOUT("Error in parsing, init or data expected"); return false;}
        if (command == "init") {
            load_data = false;
            double init_val=0;
            ss >> init_val;
            if (!ss){EOUT("Error in parsing, init <val> expected."); return false;}
            if (!this->init_array(init_val)) {return false;}
        } else {
            load_data = true;
            std::string data_qual;
            ss >> data_qual;
            if (!ss || !(data_qual=="ascii" || data_qual=="binary")){EOUT("Error in parsing, [data ascii] or [data binary] expected."); return false;}
            if (data_qual=="ascii") data_format_ascii=true; else data_format_ascii=false;
            unsigned long file_q_array_size = 0;
            unsigned int file_action_size = 0;
            ss >> file_q_array_size >> file_action_size;
            if (!ss) {EOUT("Error in parsing, [data ascii <number of states> <number of action>] or [data binary <number of states> <number of action>] expected."); return false;}
            if (file_q_array_size!=this->q_array_size || file_action_size!=this->actions.size()) {EOUT("Error in parsing, dimensions of qtable not matching definition."); return false;}
        }
    }

    if (!load_data) {
        in.close();
        return true;
    }

    if (data_format_ascii) {
        // load ascii
         for (unsigned long int i=0; i<this->q_array_size; i++) {
            getline(in, line);
            if (!in) {EOUT("Error in parsing file, not enough data lines."); return false;}
            std::stringstream ss(line);
            unsigned long int file_id;
            ss >> file_id;
            for (unsigned int j=0; j<this->actions.size(); j++) ss >> this->q_array[i][j];
            if (!ss) {EOUT("Error in parsing file, not enough data in line <state id> <qvalaction1><qvalaction2>..."); return false;}
            assert (file_id==i);
         }
    } else {
        EOUT("NYI");
        return false;
    }

    in.close();

    return true;
}

bool QTable::save( const char* qtable_definition_fname , bool ascii )
{
    assert(this->q_array!=0);
    std::ofstream out(qtable_definition_fname);

    if (!out) {
        EOUT("Can not open file " << qtable_definition_fname << " for writing!");
        return false;
    }

    out << "# This is a qtable definition file\n"
            << "# format: \n"
            << "# dimensions <xdim> <udim>\n"
            << "# xsupports <min> <max> <number>    #support points for state dim 0\n"
            << "# xsupports <min> <max> <number>    #support points for state dim 1\n"
            << "# ...\n"
            << "# actions <action definition>\n"
            << "# {init <init_value>} | { ascii|binary <number of states> <number of actions>\n"
            << "# data ... }\n"
            << "# data format for ascii: for every state a line with <state id><q_action0_value><q_action1_value>...\n"
            << "# data format for binary: NYI\n";

    out << "dimensions " << this->xdim << " " << this->udim << "\n";

    for (unsigned int i=0; i<xdim; i++)
        out << "xsupports " << this->state_supports_min[i] << " " << this->state_supports_max[i] << " " << this->state_supports_number[i] << "\n";

    out << "actions ";
    for (unsigned int i=0; i<this->actions.size(); i++) {
        out << "(";
        for (unsigned int j=0; j<this->udim; j++) {
            out << this->actions[i][j];
            if (j<this->udim-1) out << " ";
        }
        out << ")";
    }
    out << "\n";

    if (ascii) {
        out << "data ascii " << this->q_array_size << " " << this->actions.size() << "\n";
        for (unsigned long int i=0; i<this->q_array_size; i++) {
            out << i << " ";
            for (unsigned int j=0; j<this->actions.size(); j++) out << this->q_array[i][j] << " ";
            out << "\n";
        }
    } else {
        out << "data binary " << this->q_array_size << " " << this->actions.size() << "\n";
    }

    out.close();
    return true;
}


bool QTable::create_array()
{
    if (!this->destroy_array()) return false;
    state_mult.clear();
    state_mult.resize(this->xdim);

    q_array_size = this->state_supports_number[this->xdim-1];
    state_mult[this->xdim-1]=1;
    for (int i=this->xdim-2; i>=0; i--) {
        q_array_size *= this->state_supports_number[i];
        state_mult[i] = state_mult[i+1]*this->state_supports_number[i];
    }

    q_array = new double*[q_array_size];
    for (unsigned long i=0; i<q_array_size; i++) {
        q_array[i] = new double[this->actions.size()];
        for (unsigned int j=0; j<this->actions.size(); j++) q_array[i][j] = 0.0;
    }
    return true;
}

bool QTable::init_array(double _init_value)
{
    assert(this->q_array!=0);
    for (unsigned long i=0; i<q_array_size; i++)
        for (unsigned int j=0; j<this->actions.size(); j++) q_array[i][j] = _init_value;
    return true;
}

bool QTable::destroy_array()
{
    if (this->q_array == 0) {
        this->q_array_size = 0;
        return true;
    }

    for (unsigned long i=0; i<q_array_size; i++) {
        delete[] q_array[i];
    }
    delete[] q_array;
    q_array = 0;
    q_array_size=0;
    return true;
}
