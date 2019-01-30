{ stdenv
, mkRosPackage
, robonomics_comm 
}:

mkRosPackage rec {
  name = "${pname}-${version}";
  pname = "fuji_weather";
  version = "master";

  src = ./.;

  propagatedBuildInputs = [ robonomics_comm ];

  meta = with stdenv.lib; {
    description = "Robonomics winter school fuji lesson";
    homepage = http://github.com/airalab/robonomics-winter-school-2018;
    license = licenses.bsd3;
    maintainers = with maintainers; [ akru ];
  };
}
