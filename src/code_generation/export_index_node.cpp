
#include <acado/code_generation/export_index_node.hpp>
#include <sstream>

BEGIN_NAMESPACE_ACADO

using namespace std;


returnValue ExportIndexNode::exportDataDeclaration(	FILE* file,
													const String& _realString,
													const String& _intString,
													int _precision
													) const
{
	stringstream s;

	if (isGiven() == BT_TRUE)
		return ACADOERRORTEXT(RET_UNABLE_TO_EXPORT_CODE, "Export of given indices is not supported.");

	s << _intString.getName() << " " << getFullName().getName() << ";" << endl;

	return acadoFPrintf(file, "%s", s.str().c_str());
}


const String ExportIndexNode::get( ) const
{
	stringstream s;

	switch ( varType )
	{
	case EVT_VALUE:
		s << value;
		break;

	case EVT_VARIABLE:
		if (factor == 1)
			s << getFullName().getName();
		else
			s << getFullName().getName() << " * " << factor;

		if ( offset )
			s << " + " << offset;

		break;

	case EVT_BINARY_OPERATOR:

		s << "(" << left.get().getName() << ")";
		switch ( op )
		{
		case ESO_ADD:
			s << " + ";
			break;

		case ESO_SUBTRACT:
			s << " - ";
			break;

		case ESO_MULTIPLY:
			s << " * ";
			break;

		case ESO_DIVIDE:
			s << " / ";
			break;
		}
		s << "(" << right.get().getName() << ")";
		break;
	}

	String str( s.str().c_str() );

	return str;
}


const int ExportIndexNode::getGivenValue( ) const
{
	if (varType == EVT_VALUE)
		return value;

	if (varType == EVT_VARIABLE)
		return -1;

	switch ( op )
	{
	case ESO_ADD:
		return left.getGivenValue() + right.getGivenValue();

	case ESO_SUBTRACT:
		return left.getGivenValue() - right.getGivenValue();

	case ESO_MULTIPLY:
		return left.getGivenValue() * right.getGivenValue();

	case ESO_DIVIDE:
		return left.getGivenValue() / right.getGivenValue();

	default:
		return 0;
	}
}


BooleanType ExportIndexNode::isGiven( ) const
{
	if (varType == EVT_VALUE)
		return BT_TRUE;

	return BT_FALSE;
}


CLOSE_NAMESPACE_ACADO
