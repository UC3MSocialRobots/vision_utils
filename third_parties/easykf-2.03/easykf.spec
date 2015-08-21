Summary: Extended and Unscented Kalman Filter (C++).
Name: easykf
Version: 2.03
Release: 1
License: GPL
Group: Development/Libraries
#URL: 
Source0: %{name}-%{version}.tar.gz
BuildRoot: %{_tmppath}/%{name}-%{version}-%{release}-root

# These are rpm names, not names of pkg-config .pc files.
Requires: gsl >= 1.10
BuildRequires: gsl-devel >= 1.10

Packager: Jeremy Fix <Jeremy.Fix@gmail.com>

%description
Unscented Kalman Filter Library (C++).

%package devel
Summary: Libraries, includes to develop applications with %{name}.
Group: Development/Libraries
Requires: %{name} = %{version}

%description devel
The %{name}-devel package contains the header files for
building applications which use %{name}.


%prep
%setup -q

%build
./reconf
./configure --prefix=%{_prefix} --libdir=%{_prefix}/%{_archi_libdir}

%install
rm -rf $RPM_BUILD_ROOT
make DESTDIR=$RPM_BUILD_ROOT install

%clean
rm -rf $RPM_BUILD_ROOT

# Can't use -p /sbin/ldconfig, that gives '/sbin/ldconfig: relative path `1' used to build cache' error
%post 
umask 007
/sbin/ldconfig > /dev/null 2>&1

# Can't use -p /sbin/ldconfig, that gives '/sbin/ldconfig: relative path `0' used to build cache' error
%postun
umask 007
/sbin/ldconfig > /dev/null 2>&1


#describe all installed files here
%files
%defattr(-,root,root,-)
%{_prefix}/%{_archi_libdir}/libeasykf.*
%{_prefix}/bin/*

#describe all installed files here
%files devel
%defattr(-,root,root)
%{_prefix}/include/easykf/*.h
%{_prefix}/%{_archi_libdir}/pkgconfig/*.pc
%{_prefix}/share/doc/*

%changelog



