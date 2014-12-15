/*
---           `xmlfs' 0.0.0 (c) 1978 by Marcin 'Amok' Konarski            ---

	fs.cxx - this file is integral part of `xmlfs' project.

  i.  You may not make any changes in Copyright information.
  ii. You must attach Copyright information to any part of every copy
      of this software.

Copyright:

 You can use this software free of charge and you can redistribute its binary
 package freely but:
  1. You are not allowed to use any part of sources of this software.
  2. You are not allowed to redistribute any part of sources of this software.
  3. You are not allowed to reverse engineer this software.
  4. If you want to distribute a binary package of this software you cannot
     demand any fees for it. You cannot even demand
     a return of cost of the media or distribution (CD for example).
  5. You cannot involve this software in any commercial activity (for example
     as a free add-on to paid software or newspaper).
 This program is distributed in the hope that it will be useful, but WITHOUT
 ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 FITNESS FOR A PARTICULAR PURPOSE. Use it at your own risk.
*/

#include <atomic>

#include <cstdlib>
#include <cstring>

#include <unistd.h>
#define FUSE_USE_VERSION 29
#include <fuse.h>
#include <sys/xattr.h>

#include <yaal/config.hxx>
#include <yaal/hcore/htime.hxx>
#include <yaal/hcore/hthread.hxx>
#include <yaal/hcore/hlog.hxx>
#include <yaal/tools/hxml.hxx>
#include <yaal/tools/filesystem.hxx>
#include <yaal/tools/streamtools.hxx>
#include <yaal/tools/stringalgo.hxx>
//M_VCSID( "$Id: " __ID__ " $" )
//M_VCSID( "$Id: " __TID__ " $" )
#include "config.hxx"
#include "fs.hxx"
#include "setup.hxx"

using namespace yaal;
using namespace yaal::hcore;
using namespace yaal::tools;

namespace xmlfs {

class HFileSystem;
typedef yaal::hcore::HExceptionT<HFileSystem> HFileSystemException;

class HFileSystem {
public:
	typedef HFileSystem this_type;
	typedef yaal::hcore::HPointer<HFileSystem> ptr_t;
private:
	yaal::tools::filesystem::path_t _imagePath;
	yaal::tools::HXml _image;
	typedef yaal::u64_t handle_t;
	typedef yaal::hcore::HHashMap<handle_t, tools::HXml::HConstNodeProxy> directory_scans_t;
	directory_scans_t _directoryScans;
	bool _synced;
	yaal::hcore::HMutex _mutex;
	typedef std::atomic<u64_t> inode_generator_t;
	typedef std::atomic<u64_t> operation_id_generator_t;
	static inode_generator_t _inodeGenerator;
	static operation_id_generator_t _operationIdGenerator;
	static yaal::hcore::HString const ROOT_NODE;
	struct FILE {
		struct PROPERTY {
			static yaal::hcore::HString const INODE;
			static yaal::hcore::HString const INODE_NEXT;
			static yaal::hcore::HString const NAME;
			static yaal::hcore::HString const MODE;
			static yaal::hcore::HString const USER;
			static yaal::hcore::HString const GROUP;
			static yaal::hcore::HString const SIZE;
			static yaal::hcore::HString const SYMLINK;
			struct TIME {
				static yaal::hcore::HString const MODIFICATION;
				static yaal::hcore::HString const CHANGE;
				static yaal::hcore::HString const ACCESS;
			};
		};
		struct TYPE {
			static yaal::hcore::HString const PLAIN;
			static yaal::hcore::HString const DIRECTORY;
			static yaal::hcore::HString const SYMLINK;
			static yaal::hcore::HString const SOCKET;
			static yaal::hcore::HString const FIFO;
			struct DEVICE {
				static yaal::hcore::HString const BLOCK;
				static yaal::hcore::HString const CHARACTER;
			};
		};
		struct CONTENT {
			static yaal::hcore::HString const DATA;
			static yaal::hcore::HString const XATTR;
		};
	};
public:
	HFileSystem( yaal::hcore::HString const& imagePath_ )
		: _imagePath( imagePath_ ),
		_image(),
		_directoryScans(),
		_synced( false ),
		_mutex() {
		M_PROLOG
		if ( filesystem::exists( _imagePath ) ) {
			_image.load( tools::ensure( make_pointer<HFile>( _imagePath, HFile::OPEN::READING ) ) );
			_synced = true;
			HXml::HConstNodeProxy n( _image.get_root() );
			HXml::HNode::properties_t const& p( n.properties() );
			_inodeGenerator.store( lexical_cast<u64_t>( p.at( FILE::PROPERTY::INODE_NEXT ) ) );
		} else {
			_image.create_root( FILE::TYPE::DIRECTORY );
			HXml::HNodeProxy root( _image.get_root() );
			HTime now( HTime::TZ::LOCAL );
			HXml::HNode::properties_t& p( root.properties() );
			p.insert( make_pair( FILE::PROPERTY::TIME::MODIFICATION, now.to_string() ) );
			p.insert( make_pair( FILE::PROPERTY::TIME::CHANGE, now.to_string() ) );
			p.insert( make_pair( FILE::PROPERTY::TIME::ACCESS, now.to_string() ) );
			HString mode;
			mode.format( "%04o", get_mode() );
			p.insert( make_pair( FILE::PROPERTY::MODE, mode ) );
			p.insert( make_pair( FILE::PROPERTY::USER, to_string( getuid() ) ) );
			p.insert( make_pair( FILE::PROPERTY::GROUP, to_string( getgid() ) ) );
			p.insert( make_pair( FILE::PROPERTY::INODE, to_string( _inodeGenerator ++ ) ) );
			p.insert( make_pair( FILE::PROPERTY::INODE_NEXT, to_string( _inodeGenerator.load() ) ) );
		}
		return;
		M_EPILOG
	}
	virtual ~HFileSystem( void ) {
		M_PROLOG
		if ( ! _directoryScans.is_empty() ) {
			log( LOG_TYPE::ERROR ) << "opendir leak!" << endl;
		}
		return;
		M_DESTRUCTOR_EPILOG
	}
	void save( void ) {
		M_PROLOG
		HLock l( _mutex );
		if ( ! _synced ) {
			_image.save( tools::ensure( make_pointer<HFile>( _imagePath, HFile::OPEN::WRITING | HFile::OPEN::TRUNCATE ) ) );
			_synced = true;
		}
		return;
		M_EPILOG
	}
	handle_t opendir( tools::filesystem::path_t const& path_ ) {
		M_PROLOG
		HLock l( _mutex );
		errno = 0;
		handle_t h( _operationIdGenerator ++ );
		HXml::HConstNodeProxy n( get_node_by_path( path_ ) );
		if ( n.get_name() != FILE::TYPE::DIRECTORY ) {
			errno = ENOTDIR;
			throw HFileSystemException( "Path does not point to a directory: "_ys.append( path_ ) );
		}
		_directoryScans.insert( make_pair( h, n ) );
		return ( h );
		M_EPILOG
	}
	void releasedir( handle_t handle_ ) {
		M_PROLOG
		HLock l( _mutex );
		errno = 0;
		directory_scans_t::iterator it( _directoryScans.find( handle_ ) );
		if ( ! ( it != _directoryScans.end() ) ) {
			errno = EINVAL;
			throw HFileSystemException( "Invalid handle: "_ys.append( handle_ ) );
		}
		_directoryScans.erase( it );
		return;
		M_EPILOG
	}
	void getattr( char const* path_, struct stat* stat_ ) {
		M_PROLOG
		HLock l( _mutex );
		get_stat( get_node_by_path( path_ ), stat_ );
		errno = 0;
		return;
		M_EPILOG
	}
	void readdir( void* buf_, fuse_fill_dir_t filler_,
		off_t, struct fuse_file_info* info_ ) {
		M_PROLOG
		HLock l( _mutex );
		HXml::HConstNodeProxy const& n( _directoryScans.at( info_->fh ) );
		struct stat s;
		get_stat( n, &s );
		filler_( buf_, ".", &s, 0 );
		filler_( buf_, "..", NULL, 0 );
		for ( HXml::HConstNodeProxy const& c : n ) {
			get_stat( c, &s );
			filler_( buf_, c.properties().at( FILE::PROPERTY::NAME ).c_str(), &s, 0 );
		}
		errno = 0;
		return;
		M_EPILOG
	}
	int access( char const* path_, int mode_ ) {
		M_PROLOG
		HLock l( _mutex );
		HXml::HNodeProxy n( get_node_by_path( path_ ) );
		HXml::HNode::properties_t& p( n.properties() );
		mode_t mode( get_mode( n ) );
		uid_t u( get_user( n ) );
		gid_t g( get_group( n ) );
		int accessBit( 6 );
		if ( getuid() != u ) {
			accessBit -= 3;
		}
		if ( getgid() != g ) {
			accessBit -= 3;
		}
		mode >>= accessBit;
		bool ok( false );
		do {
			if ( mode_ & R_OK ) {
				if ( ! ( mode & S_IROTH ) ) {
					break;
				}
			}
			if ( mode_ & W_OK ) {
				if ( ! ( mode & S_IWOTH ) ) {
					break;
				}
			}
			if ( mode_ & X_OK ) {
				if ( ! ( mode & S_IXOTH ) ) {
					break;
				}
			}
			ok = true;
		} while ( false );
		return ( ok ? 0 : EACCES );
		M_EPILOG
	}
	int getxattr( char const* path_, HString const& name_, char* buffer_, size_t size_ ) {
		M_PROLOG
		HLock l( _mutex );
		HXml::HConstNodeProxy n( get_node_by_path( path_ ) );
		int pos( static_cast<int>( name_.find( '.' ) ) );
		if ( pos == HString::npos ) {
			errno = EINVAL;
			throw HFileSystemException( "Invalid attribute name: "_ys.append( name_ ) );
		}
		bool ok( false );
		for ( HXml::HConstNodeProxy c : n ) {
			if ( c.get_name() == FILE::CONTENT::XATTR ) {
				n = c;
				ok = true;
				break;
			}
		}
		if ( ok ) {
			ok = false;
			HString ns( name_.left( pos ) );
			for ( HXml::HConstNodeProxy c : n ) {
				if ( c.get_name() == ns ) {
					ok = true;
					n = c;
					break;
				}
			}
		}
		if ( ok ) {
			ok = false;
			HString name( name_.mid( pos + 1 ) );
			for ( HXml::HConstNodeProxy c : n ) {
				if ( c.get_name() == name ) {
					ok = true;
					n = c;
					break;
				}
			}
		}
		int ret( 0 );
		if ( ok ) {
			errno = 0;
			ret = static_cast<int>( n.get_value().get_size() );
			if ( ret <= static_cast<int>( size_ ) ) {
				::memcpy( buffer_, n.get_value().c_str(), ret );
			} else if ( static_cast<int>( size_ ) > 0 ) {
				ret = ERANGE;
			}
		} else {
			ret = ENODATA;
		}
		return ( ret );
		M_EPILOG
	}
	int listxattr( char const* path_, char* buffer_, size_t size_ ) {
		M_PROLOG
		HLock l( _mutex );
		HXml::HConstNodeProxy n( get_node_by_path( path_ ) );
		bool ok( false );
		for ( HXml::HConstNodeProxy c : n ) {
			if ( c.get_name() == FILE::CONTENT::XATTR ) {
				n = c;
				ok = true;
				break;
			}
		}
		int ret( 0 );
		if ( ok ) {
			typedef HArray<HString const*> names_t;
			names_t names;
			int total( 0 );
			for ( HXml::HConstNodeProxy ns : n ) {
				for ( HXml::HConstNodeProxy attr : ns ) {
					names.push_back( &attr.get_name() );
					total += static_cast<int>( attr.get_name().get_size() + 1 );
				}
			}
			ret = total;
			if ( total <= static_cast<int>( size_ ) ) {
				int offset( 0 );
				for ( HString const* s : names ) {
					::memcpy( buffer_ + offset, s->c_str(), s->get_size() );
					buffer_[offset] = 0;
					offset += static_cast<int>( s->get_size() );
					++ offset;
				}
				errno = 0;
			} else if ( size_ > 0 ) {
				ret = ERANGE;
			}
		} else {
			errno = 0;
		}
		return ( ret );
		M_EPILOG
	}
private:
	HFileSystem( HFileSystem const& ) = delete;
	HFileSystem& operator = ( HFileSystem const& ) = delete;
	HXml::HNodeProxy get_node_by_path( tools::filesystem::path_t const& path_ ) {
		M_PROLOG
		typedef HArray<HString> components_t;
		components_t path( string::split<components_t>( filesystem::normalize_path( path_ ), "/", HTokenizer::SKIP_EMPTY ) );
		HXml::HNodeProxy n( _image.get_root() );
		for ( HString const& name : path ) {
			if ( ( name == "." ) || ( name == ".." ) ) {
				errno = EINVAL;
				throw HFileSystemException( "Bogus path component: "_ys.append( path_ ).append( ": " ).append( name ) );
			}
			bool found( false );
			for ( HXml::HNodeProxy c : n ) {
				HXml::HNode::properties_t::const_iterator it( c.properties().find( FILE::PROPERTY::NAME ) );
				if ( ( it != c.properties().end() ) && ( it->second == name ) ) {
					n = c;
					found = true;
					break;
				}
			}
			if ( ! found ) {
				errno = ENOENT;
				throw HFileSystemException( "No such file or directory: "_ys.append( path_ ) );
			}
		}
		return ( n );
		M_EPILOG
	}
	int get_hard_link_count( HXml::HConstNodeProxy const& n_ ) {
		int hlc( 2 );
		for ( HXml::HConstNodeProxy const& c : n_ ) {
			if ( c.get_name() == FILE::TYPE::DIRECTORY ) {
				++ hlc;
			}
		}
		return ( hlc );
	}
	uid_t get_user( HXml::HConstNodeProxy const& n_ ) const {
		M_PROLOG
		return ( lexical_cast<uid_t>( n_.properties().at( FILE::PROPERTY::USER ) ) );
		M_EPILOG
	}
	uid_t get_group( HXml::HConstNodeProxy const& n_ ) const {
		M_PROLOG
		return ( lexical_cast<uid_t>( n_.properties().at( FILE::PROPERTY::GROUP ) ) );
		M_EPILOG
	}
	uid_t get_mode( HXml::HConstNodeProxy const& n_ ) const {
		M_PROLOG
		return ( lexical_cast<mode_t>( n_.properties().at( FILE::PROPERTY::MODE ) ) );
		M_EPILOG
	}
	void get_stat( HXml::HConstNodeProxy const& n_, struct stat* stat_ ) {
		M_PROLOG
		::memset( stat_, 0, sizeof ( *stat_ ) );
		HXml::HNode::properties_t const& p( n_.properties() );
		HString type( n_.get_name() );
		if ( type == FILE::TYPE::PLAIN ) {
			stat_->st_mode = S_IFREG;
			stat_->st_size = lexical_cast<i64_t>( p.at( FILE::PROPERTY::SIZE ) );
			stat_->st_nlink = 1;
		} else if ( type == FILE::TYPE::DIRECTORY ) {
			stat_->st_mode = S_IFDIR;
			stat_->st_nlink = get_hard_link_count( n_ );
			stat_->st_size = ( stat_->st_nlink ) * 160;
		} else if ( type == FILE::TYPE::SYMLINK ) {
			stat_->st_mode = S_IFLNK;
			stat_->st_size = p.at( FILE::PROPERTY::SYMLINK ).get_length();
		} else if ( type == FILE::TYPE::SOCKET ) {
			stat_->st_mode = S_IFSOCK;
		} else if ( type == FILE::TYPE::FIFO ) {
			stat_->st_mode = S_IFIFO;
		} else if ( type == FILE::TYPE::DEVICE::BLOCK ) {
			stat_->st_mode = S_IFBLK;
		} else if ( type == FILE::TYPE::DEVICE::CHARACTER ) {
			stat_->st_mode = S_IFCHR;
		}
		stat_->st_ino = lexical_cast<ino_t>( p.at( FILE::PROPERTY::INODE ) );
		stat_->st_uid = lexical_cast<uid_t>( p.at( FILE::PROPERTY::USER ) );
		stat_->st_gid = lexical_cast<uid_t>( p.at( FILE::PROPERTY::GROUP ) );
		stat_->st_mtim.tv_sec = HTime( p.at( FILE::PROPERTY::TIME::MODIFICATION ) ).raw() - HTime::SECONDS_TO_UNIX_EPOCH;
		stat_->st_ctim.tv_sec = HTime( p.at( FILE::PROPERTY::TIME::CHANGE ) ).raw() - HTime::SECONDS_TO_UNIX_EPOCH;
		stat_->st_atim.tv_sec = HTime( p.at( FILE::PROPERTY::TIME::ACCESS ) ).raw() - HTime::SECONDS_TO_UNIX_EPOCH;
		stat_->st_mode |= lexical_cast<mode_t>( p.at( FILE::PROPERTY::MODE ) );
		return;
		M_EPILOG
	}
	mode_t get_umask( void ) const {
		static char const SYSCALL_FAILURE[] = "syscall failure - bailng out";
		mode_t const lockOutUmask( S_IRUSR | S_IWUSR | S_IXUSR | S_IRGRP | S_IWGRP | S_IXGRP | S_IROTH | S_IWOTH | S_IXOTH );
		mode_t curUmask( ::umask( lockOutUmask ) );
		if ( ::umask( curUmask ) != lockOutUmask ) {
			log( LOG_TYPE::ERROR ) << SYSCALL_FAILURE << endl;
			exit( 1 );
		}
		return ( curUmask );
	}
	mode_t get_mode( void ) const {
		mode_t const fullMode( S_IRUSR | S_IWUSR | S_IXUSR | S_IRGRP | S_IWGRP | S_IXGRP | S_IROTH | S_IWOTH | S_IXOTH );
		return ( fullMode & ~get_umask() );
	}
};
HFileSystem::operation_id_generator_t HFileSystem::_inodeGenerator( 0 );
HFileSystem::operation_id_generator_t HFileSystem::_operationIdGenerator( 0 );
yaal::hcore::HString const HFileSystem::ROOT_NODE( "root" );
yaal::hcore::HString const HFileSystem::FILE::PROPERTY::INODE( "id" );
yaal::hcore::HString const HFileSystem::FILE::PROPERTY::INODE_NEXT( "inode_next" );
yaal::hcore::HString const HFileSystem::FILE::PROPERTY::NAME( "name" );
yaal::hcore::HString const HFileSystem::FILE::PROPERTY::MODE( "mode" );
yaal::hcore::HString const HFileSystem::FILE::PROPERTY::USER( "user" );
yaal::hcore::HString const HFileSystem::FILE::PROPERTY::GROUP( "group" );
yaal::hcore::HString const HFileSystem::FILE::PROPERTY::SIZE( "size" );
yaal::hcore::HString const HFileSystem::FILE::PROPERTY::TIME::MODIFICATION( "mtime" );
yaal::hcore::HString const HFileSystem::FILE::PROPERTY::TIME::CHANGE( "ctime" );
yaal::hcore::HString const HFileSystem::FILE::PROPERTY::TIME::ACCESS( "atime" );
yaal::hcore::HString const HFileSystem::FILE::PROPERTY::SYMLINK( "symlink" );
yaal::hcore::HString const HFileSystem::FILE::TYPE::PLAIN( "plain" );
yaal::hcore::HString const HFileSystem::FILE::TYPE::DIRECTORY( "dir" );
yaal::hcore::HString const HFileSystem::FILE::TYPE::SYMLINK( "symlink" );
yaal::hcore::HString const HFileSystem::FILE::TYPE::SOCKET( "socket" );
yaal::hcore::HString const HFileSystem::FILE::TYPE::FIFO( "fifo" );
yaal::hcore::HString const HFileSystem::FILE::TYPE::DEVICE::BLOCK( "block_device" );
yaal::hcore::HString const HFileSystem::FILE::TYPE::DEVICE::CHARACTER( "character_device" );
yaal::hcore::HString const HFileSystem::FILE::CONTENT::DATA( "data" );
yaal::hcore::HString const HFileSystem::FILE::CONTENT::XATTR( "xattr" );

HFileSystem::ptr_t _fs_;

namespace {

int getattr( char const* path_, struct stat* stat_ ) {
	if ( setup._debug ) {
		log_trace << path_ << endl;
	}
	int ret( 0 );
	try {
		_fs_->getattr( path_, stat_ );
	} catch ( HException const& e ) {
		ret = errno;
		log( LOG_TYPE::ERROR ) << e.what() << ", " << errno << endl;
	}
	return ( ret );
}

int readlink( char const*, char*, size_t ) {
	log << __PRETTY_FUNCTION__ << endl;
	return ( -1 );
}

int open( char const*, struct fuse_file_info* ) {
	log << __PRETTY_FUNCTION__ << endl;
	return ( -1 );
}

int mknod( char const*, mode_t, dev_t ) {
	log << __PRETTY_FUNCTION__ << endl;
	return ( -1 );
}

int mkdir( char const*, mode_t ) {
	log << __PRETTY_FUNCTION__ << endl;
	return ( -1 );
}

int unlink( char const* ) {
	log << __PRETTY_FUNCTION__ << endl;
	return ( -1 );
}

int rmdir( char const* ) {
	log << __PRETTY_FUNCTION__ << endl;
	return ( -1 );
}

int symlink( char const*, char const* ) {
	log << __PRETTY_FUNCTION__ << endl;
	return ( -1 );
}

int rename( char const*, char const* ) {
	log << __PRETTY_FUNCTION__ << endl;
	return ( -1 );
}

int link( char const*, char const* ) {
	log << __PRETTY_FUNCTION__ << endl;
	return ( -1 );
}

int chmod( char  const*, mode_t ) {
	log << __PRETTY_FUNCTION__ << endl;
	return ( -1 );
}

int chown( char const*, uid_t, gid_t ) {
	log << __PRETTY_FUNCTION__ << endl;
	return ( -1 );
}

int truncate( char const*, off_t ) {
	log << __PRETTY_FUNCTION__ << endl;
	return ( -1 );
}

int read( char  const*, char*, size_t, off_t,
	struct fuse_file_info* ) {
	log << __PRETTY_FUNCTION__ << endl;
	return ( -1 );
}

int write( char const*, char const*, size_t, off_t, struct fuse_file_info* ) {
	log << __PRETTY_FUNCTION__ << endl;
	return ( -1 );
}

int statfs( char  const*, struct statvfs * ) {
	log << __PRETTY_FUNCTION__ << endl;
	return ( -1 );
}

int flush( char const*, struct fuse_file_info* ) {
	log << __PRETTY_FUNCTION__ << endl;
	return ( -1 );
}

int release( char const*, struct fuse_file_info* ) {
	log << __PRETTY_FUNCTION__ << endl;
	return ( -1 );
}

int fsync( char const*, int, struct fuse_file_info* ) {
	log << __PRETTY_FUNCTION__ << endl;
	return ( -1 );
}

int setxattr( char const*, char const*, char const*, size_t, int ) {
	log << __PRETTY_FUNCTION__ << endl;
	return ( -1 );
}

int getxattr( char const* path_, char const* name_, char* buffer_, size_t size_ ) {
	if ( setup._debug ) {
		log_trace << path_ << endl;
	}
	int ret( 0 );
	try {
		ret = _fs_->getxattr( path_, name_, buffer_, size_ );
	} catch ( HException const& e ) {
		ret = errno;
		log( LOG_TYPE::ERROR ) << e.what() << endl;
	}
	return ( ret );
}

int listxattr( char const* path_, char* buffer_, size_t size_ ) {
	if ( setup._debug ) {
		log_trace << endl;
	}
	int ret( 0 );
	try {
		ret = _fs_->listxattr( path_, buffer_, size_ );
	} catch ( HException const& e ) {
		ret = errno;
		log( LOG_TYPE::ERROR ) << e.what() << endl;
	}
	return ( ret );
}

int removexattr( char const*, char const* ) {
	log << __PRETTY_FUNCTION__ << endl;
	return ( -1 );
}

int opendir( char const* path_, struct fuse_file_info* info_ ) {
	if ( setup._debug ) {
		log_trace << path_ << endl;
	}
	int ret( 0 );
	try {
		info_->fh = _fs_->opendir( path_ );
	} catch ( HException const& e ) {
		ret = errno;
		log( LOG_TYPE::ERROR ) << e.what() << endl;
	}
	return ( ret );
}

int readdir( char const*, void* buffer_, fuse_fill_dir_t filler_,
	off_t offset_, struct fuse_file_info* info_ ) {
	if ( setup._debug ) {
		log_trace << info_->fh << endl;
	}
	int ret( 0 );
	try {
		_fs_->readdir( buffer_, filler_, offset_, info_ );
	} catch ( HException const& e ) {
		ret = errno;
		log( LOG_TYPE::ERROR ) << e.what() << endl;
	}
	return ( ret );
}

int releasedir( char const* path_, struct fuse_file_info* info_ ) {
	if ( setup._debug ) {
		log_trace << path_ << endl;
	}
	int ret( 0 );
	try {
		_fs_->releasedir( info_->fh );
	} catch ( HException const& e ) {
		ret = errno;
		log( LOG_TYPE::ERROR ) << e.what() << endl;
	}
	return ( ret );
}

int fsyncdir( char const*, int, struct fuse_file_info* ) {
	log << __PRETTY_FUNCTION__ << endl;
	return ( -1 );
}

void* init( struct fuse_conn_info* info_ ) {
	if ( setup._debug ) {
		log_trace << endl;
	}
	info_->async_read = false;
	info_->max_readahead = 0;
	info_->want = 0;
	try {
		_fs_ = make_pointer<HFileSystem>( setup._fsFilePath );
	} catch ( HException const& e ) {
		log( LOG_TYPE::ERROR ) << e.what() << endl;
		exit( 1 );
	}
	return ( nullptr );
}

void destroy( void * ) {
	if ( setup._debug ) {
		log_trace << endl;
	}
	try {
		_fs_->save();
		_fs_.reset();
	} catch ( HException const& e ) {
		log( LOG_TYPE::ERROR ) << e.what() << endl;
		exit( 1 );
	}
	return;
}

int access( char const* path_, int mode_ ) {
	if ( setup._debug ) {
		log_trace << endl;
	}
	int ret( 0 );
	try {
		ret = _fs_->access( path_, mode_ );
	} catch ( HException const& e ) {
		ret = errno;
		log( LOG_TYPE::ERROR ) << e.what() << endl;
	}
	return ( ret );
}

int create( char const*, mode_t, struct fuse_file_info* ) {
	log << __PRETTY_FUNCTION__ << endl;
	return ( -1 );
}

int ftruncate( char const*, off_t, struct fuse_file_info* ) {
	log << __PRETTY_FUNCTION__ << endl;
	return ( -1 );
}

int fgetattr( char const*, struct stat*, struct fuse_file_info* ) {
	log << __PRETTY_FUNCTION__ << endl;
	return ( -1 );
}

int lock( char const*, struct fuse_file_info*, int, struct flock* ) {
	log << __PRETTY_FUNCTION__ << endl;
	return ( -1 );
}

int utimens( char const*, const struct timespec[2] ) {
	log << __PRETTY_FUNCTION__ << endl;
	return ( -1 );
}

int bmap( char const*, size_t, uint64_t* ) {
	log << __PRETTY_FUNCTION__ << endl;
	return ( -1 );
}

int ioctl( char const*, int, void*, struct fuse_file_info*, int unsigned, void* ) {
	log << __PRETTY_FUNCTION__ << endl;
	return ( -1 );
}

int poll( char const*, struct fuse_file_info*, struct fuse_pollhandle*, int unsigned* ) {
	log << __PRETTY_FUNCTION__ << endl;
	return ( -1 );
}

int write_buf( char const*, struct fuse_bufvec*, off_t, struct fuse_file_info* ) {
	log << __PRETTY_FUNCTION__ << endl;
	return ( -1 );
}

int read_buf( char const*, struct fuse_bufvec**, size_t, off_t, struct fuse_file_info* ) {
	log << __PRETTY_FUNCTION__ << endl;
	return ( -1 );
}

int flock( char const*, struct fuse_file_info*, int ) {
	log << __PRETTY_FUNCTION__ << endl;
	return ( -1 );
}

int fallocate( char const*, int, off_t, off_t, struct fuse_file_info* ) {
	log << __PRETTY_FUNCTION__ << endl;
	return ( -1 );
}

}

struct fuse_operations xmlfsFuse;

int main( int argc_, char** argv_ ) {
	::memset( &xmlfsFuse, 0, sizeof ( xmlfsFuse ) );
	xmlfsFuse.getattr     = &getattr;
	xmlfsFuse.readlink    = &readlink;
	xmlfsFuse.getdir      = nullptr;
	xmlfsFuse.mknod       = &mknod;
	xmlfsFuse.mkdir       = &mkdir;
	xmlfsFuse.unlink      = &unlink;
	xmlfsFuse.rmdir       = &rmdir;
	xmlfsFuse.symlink     = &symlink;
	xmlfsFuse.rename      = &rename;
	xmlfsFuse.link        = &link;
	xmlfsFuse.chmod       = &chmod;
	xmlfsFuse.chown       = &chown;
	xmlfsFuse.truncate    = &truncate;
	xmlfsFuse.utime       = nullptr;
	xmlfsFuse.open        = &open;
	xmlfsFuse.read        = &read;
	xmlfsFuse.write       = &write;
	xmlfsFuse.statfs      = &statfs;
	xmlfsFuse.flush       = &flush;
	xmlfsFuse.release     = &release;
	xmlfsFuse.fsync       = &fsync;
	xmlfsFuse.setxattr    = &setxattr;
	xmlfsFuse.getxattr    = &getxattr;
	xmlfsFuse.listxattr   = &listxattr;
	xmlfsFuse.removexattr = &removexattr;
	xmlfsFuse.opendir     = &opendir;
	xmlfsFuse.readdir     = &readdir;
	xmlfsFuse.releasedir  = &releasedir;
	xmlfsFuse.fsyncdir    = &fsyncdir;
	xmlfsFuse.init        = &init;
	xmlfsFuse.destroy     = &destroy;
	xmlfsFuse.access      = &access;
	xmlfsFuse.create      = &create;
	xmlfsFuse.ftruncate   = &ftruncate;
	xmlfsFuse.fgetattr    = &fgetattr;
	xmlfsFuse.lock        = &lock;
	xmlfsFuse.utimens     = &utimens;
	xmlfsFuse.bmap        = &bmap;
	xmlfsFuse.ioctl       = &ioctl;
	xmlfsFuse.poll        = &poll;
	xmlfsFuse.write_buf   = &write_buf;
	xmlfsFuse.read_buf    = &read_buf;
	xmlfsFuse.flock       = &flock;
	xmlfsFuse.fallocate   = &fallocate;
	return ( fuse_main( argc_, argv_, &xmlfsFuse, nullptr ) );
}

}

