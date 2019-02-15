{ stdenv
, mkRosPackage
, robonomics_comm 
}:

mkRosPackage rec {
  name = "${pname}-${version}";
  pname = "turtlesim_aira";
  version = "master";

  src = ./.;

  propagatedBuildInputs = [ robonomics_comm ];

  meta = with stdenv.lib; {
    description = "turtlesim_aira package";
    homepage = https://github.com/airalab/robonomics-winter-school-2018;
    license = licenses.bsd3;
    maintainers = [ ];
  };
}
