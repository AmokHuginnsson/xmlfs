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
#include <yaal/hcore/hstack.hxx>
#include <yaal/hcore/htime.hxx>
#include <yaal/hcore/hthread.hxx>
#include <yaal/hcore/hlog.hxx>
#include <yaal/tools/hxml.hxx>
#include <yaal/tools/filesystem.hxx>
#include <yaal/tools/hstringstream.hxx>
#include <yaal/tools/streamtools.hxx>
#include <yaal/tools/stringalgo.hxx>
#include <yaal/tools/hmemory.hxx>
#include <yaal/tools/base64.hxx>
M_VCSID( "$Id: " __ID__ " $" )
//M_VCSID( "$Id: " __TID__ " $" )
#include "config.hxx"
#include "fs.hxx"
#include "setup.hxx"

using namespace yaal;
using namespace yaal::hcore;
using namespace yaal::tools;
using namespace yaal::tools::filesystem;

namespace xmlfs {

class HFileSystem;
typedef yaal::hcore::HExceptionT<HFileSystem> HFileSystemException;

class HFileSystem final {
public:
	typedef HFileSystem this_type;
	typedef yaal::hcore::HPointer<HFileSystem> ptr_t;
	static int const NO_OWNER = -1;
	class HDescriptor final {
		tools::HXml::HNodeProxy _node;
		yaal::hcore::HChunk _data;
		int _size;
		yaal::hcore::HChunk _encoded;
		pid_t _owner;
	public:
		HDescriptor( tools::HXml::HNodeProxy const& node_ )
			: _node( node_ )
			, _data()
			, _size()
			, _encoded()
			, _owner( NO_OWNER ) {
			if ( is_plain( _node ) ) {
				_size = lexical_cast<int>( _node.properties().at( FILE::PROPERTY::SIZE ) );
			} else {
				_size = get_hard_link_count( _node );
			}
			return;
		}
		HDescriptor( HDescriptor&& ) = default;
		HDescriptor& operator = ( HDescriptor&& ) = default;
		HXml::HNodeProxy& node( void ) {
			return ( _node );
		}
		HXml::HConstNodeProxy const& node( void ) const {
			return ( _node );
		}
		int size( void ) const {
			return ( _size );
		}
		void flush( void ) {
			M_PROLOG
			if ( is_plain( _node ) ) {
				int oldSize( lexical_cast<int>( _node.properties().at( FILE::PROPERTY::SIZE ) ) );
				HXml::HNodeProxy value;
				if ( oldSize > 0 ) {
					HXml::HIterator valueIt = _node.begin();
					if ( _size > 0 ) {
						value = *valueIt;
					} else {
						_node.remove_node( valueIt );
					}
				} else {
					if ( _size > 0 ) {
						value = *_node.add_node( HXml::HNode::TYPE::CONTENT );
					}
				}
				if ( _size > 0 ) {
					HMemoryProvider srcMem( _data, _size );
					HMemoryProvider dstMem( _encoded, 0 );
					HMemory src( srcMem );
					HMemory dst( dstMem );
					base64::encode( src, dst );
					value.set_value( _encoded.get<char>() );
				}
				_node.properties()[ FILE::PROPERTY::SIZE ] = _size;
			}
			return;
			M_EPILOG
		}
		void lock( int cmd_, struct flock* lock_ ) {
			if ( _owner > NO_OWNER ) {
				if ( cmd_ == F_GETLK ) {
					lock_->l_pid = _owner;
					lock_->l_type = F_WRLCK;
					lock_->l_whence = SEEK_SET;
					lock_->l_start = 0;
					lock_->l_len = _size;
				} else {
					if ( lock_->l_pid != _owner ) {
						throw HFileSystemException( "Locked by another process: "_ys.append( _owner ), -EACCES );
					} else if ( lock_->l_type != F_UNLCK ) {
						throw HFileSystemException( "File already locked", -EINVAL );
					} else {
						_owner = NO_OWNER;
					}
				}
			} else {
				switch ( cmd_ ) {
					case ( F_GETLK ): {
						lock_->l_type = F_UNLCK;
					} break;
					case ( F_SETLK ):
					case ( F_SETLKW ): {
						_owner = lock_->l_pid;
					} break;
					default: {
						throw HFileSystemException( "Invalid lock command.", -EINVAL );
					}
				}
			}
		}
	private:
		HDescriptor( HDescriptor const& ) = delete;
		HDescriptor& operator = ( HDescriptor const& ) = delete;
	};
private:
	yaal::tools::filesystem::path_t _imagePath;
	yaal::tools::HXml _image;
	dev_t _devId;
	typedef yaal::u64_t handle_t;
	typedef yaal::hcore::HHashMap<handle_t, HDescriptor> descriptors_t;
	typedef yaal::hcore::HStack<handle_t> available_descriptors_t;
	descriptors_t _descriptors;
	bool _synced;
	typedef std::atomic<u64_t> inode_generator_t;
	typedef std::atomic<handle_t> descriptor_generator_t;
	inode_generator_t _inodeGenerator;
	descriptor_generator_t _descriptorGenerator;
	available_descriptors_t _availableDescriptors;
	mutable yaal::hcore::HMutex _mutex;
	static yaal::hcore::HString const ROOT_NODE;
	static blksize_t const BLOCK_SIZE = 512;
	static handle_t const INVALID_HANDLE = static_cast<handle_t>( -1 );
	struct FILE {
		struct PROPERTY {
			static yaal::hcore::HString const DEV_ID;
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
		: _imagePath( imagePath_ )
		, _image()
		, _devId( 0 )
		, _descriptors()
		, _synced( false )
		, _inodeGenerator( 0 )
		, _descriptorGenerator( 0 )
		, _availableDescriptors()
		, _mutex() {
		M_PROLOG
		if ( filesystem::exists( _imagePath ) ) {
			_image.load( tools::ensure( make_pointer<HFile>( _imagePath, HFile::OPEN::READING ) ) );
			_synced = true;
			HXml::HConstNodeProxy n( _image.get_root() );
			HXml::HNode::properties_t const& p( n.properties() );
			_inodeGenerator.store( lexical_cast<u64_t>( p.at( FILE::PROPERTY::INODE_NEXT ) ) );
			_devId = lexical_cast<dev_t>( p.at( FILE::PROPERTY::DEV_ID ) );
		} else {
			log( LOG_LEVEL::WARNING ) << "Creating new, empty file system at: " << _imagePath << endl;
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
			_inodeGenerator.store( 1 );
			_devId = randomizer_helper::make_randomizer()();
			p.insert( make_pair( FILE::PROPERTY::INODE, to_string( _inodeGenerator ++ ) ) );
			p.insert( make_pair( FILE::PROPERTY::INODE_NEXT, to_string( _inodeGenerator.load() ) ) );
			_synced = false;
		}
		return;
		M_EPILOG
	}
	~HFileSystem( void ) {
		M_PROLOG
		if ( ! _descriptors.is_empty() ) {
			log( LOG_LEVEL::ERROR ) << "descriptor leak!" << endl;
		}
		return;
		M_DESTRUCTOR_EPILOG
	}
	void save( void ) {
		M_PROLOG
		HLock l( _mutex );
		if ( ! _synced ) {
			HXml::HNodeProxy n( _image.get_root() );
			n.properties()[FILE::PROPERTY::INODE_NEXT] = _inodeGenerator.load();
			_image.save( tools::ensure( make_pointer<HFile>( _imagePath, HFile::OPEN::WRITING | HFile::OPEN::TRUNCATE ) ), true );
			_synced = true;
		}
		return;
		M_EPILOG
	}
	handle_t opendir( tools::filesystem::path_t const& path_ ) {
		M_PROLOG
		HLock l( _mutex );
		handle_t h( create_handle() );
		HXml::HNodeProxy n( get_node_by_path( path_ ) );
		if ( n.get_name() != FILE::TYPE::DIRECTORY ) {
			throw HFileSystemException( "Path does not point to a directory: "_ys.append( path_ ), -ENOTDIR );
		}
		_descriptors.insert( descriptors_t::value_type( h, HDescriptor( n ) ) );
		return ( h );
		M_EPILOG
	}
	void releasedir( handle_t handle_ ) {
		M_PROLOG
		HLock l( _mutex );
		_descriptors.erase( descriptor_iterator( handle_ ) );
		release_handle( handle_ );
		return;
		M_EPILOG
	}
	void release( handle_t handle_ ) {
		M_PROLOG
		HLock l( _mutex );
		_descriptors.erase( descriptor_iterator( handle_ ) );
		release_handle( handle_ );
		return;
		M_EPILOG
	}
	void getattr( char const* path_, struct stat* stat_ ) const {
		M_PROLOG
		HLock l( _mutex );
		HXml::HConstNodeProxy n( get_node_by_path( path_ ) );
		get_stat( n, stat_ );
		return;
		M_EPILOG
	}
	void fgetattr( handle_t handle_, struct stat* stat_ ) const {
		M_PROLOG
		HLock l( _mutex );
		get_stat( descriptor( handle_ ).node(), stat_ );
		return;
		M_EPILOG
	}
	void readdir( void* buf_, fuse_fill_dir_t filler_,
		off_t, handle_t handle_ ) const {
		M_PROLOG
		HLock l( _mutex );
		HXml::HConstNodeProxy const& n( descriptor( handle_ ).node() );
		struct stat s;
		get_stat( n, &s );
		filler_( buf_, ".", &s, 0 );
		filler_( buf_, "..", NULL, 0 );
		for ( HXml::HConstNodeProxy const& c : n ) {
			HString const& name( c.properties().at( FILE::PROPERTY::NAME ) );
			get_stat( c, &s );
			if ( filler_( buf_, name.c_str(), &s, 0 ) ) {
				throw HFileSystemException( "Directory buffer is full." );
			}
		}
		return;
		M_EPILOG
	}
	int access( char const* path_, int mode_ ) const {
		M_PROLOG
		HLock l( _mutex );
		HXml::HConstNodeProxy n( get_node_by_path( path_ ) );
		return ( have_access( n, mode_ ) ? 0 : -EACCES );
		M_EPILOG
	}
	void mkdir( tools::filesystem::path_t const& path_, mode_t mode_ ) {
		M_PROLOG
		HLock l( _mutex );
		path_t bname( basename( path_ ) );
		path_t dname( dirname( path_ ) );
		HXml::HNodeProxy p( get_node_by_path( dname ) );
		if ( p.get_name() != FILE::TYPE::DIRECTORY ) {
			throw HFileSystemException( "Path does not point to a directory: "_ys.append( dname ), -ENOTDIR );
		}
		if ( ! have_access( p, W_OK | X_OK ) ) {
			throw HFileSystemException( "You have no permission to modify parent directory.", -EACCES );
		}
		create_node( p, FILE::TYPE::DIRECTORY, bname, mode_ );
		_synced = false;
		M_EPILOG
	}
	handle_t create( tools::filesystem::path_t const& path_, mode_t mode_ ) {
		path_t bname( basename( path_ ) );
		path_t dname( dirname( path_ ) );
		HXml::HNodeProxy p( get_node_by_path( dname ) );
		if ( p.get_name() != FILE::TYPE::DIRECTORY ) {
			throw HFileSystemException( "Path does not point to a directory: "_ys.append( dname ), -ENOTDIR );
		}
		if ( ! have_access( p, W_OK | X_OK ) ) {
			throw HFileSystemException( "You have no permission to modify parent directory.", -EACCES );
		}
		HXml::HNodeProxy n( create_node( p, FILE::TYPE::PLAIN, bname, mode_ ) );
		handle_t h( create_handle() );
		_descriptors.insert( make_pair( h, n ) );
		_synced = false;
		return ( h );
	}
	int getxattr( char const* path_, HString const& name_, char* buffer_, size_t size_ ) const {
		M_PROLOG
		HLock l( _mutex );
		HXml::HConstNodeProxy n( get_node_by_path( path_ ) );
		int pos( static_cast<int>( name_.find( '.' ) ) );
		if ( pos == HString::npos ) {
			throw HFileSystemException( "Invalid attribute name: "_ys.append( name_ ), -EINVAL );
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
			ret = static_cast<int>( n.get_value().get_size() );
			if ( ret <= static_cast<int>( size_ ) ) {
				::memcpy( buffer_, n.get_value().c_str(), static_cast<int unsigned>( ret ) );
			} else if ( static_cast<int>( size_ ) > 0 ) {
				ret = -ERANGE;
			}
		} else {
			ret = -ENODATA;
		}
		return ( ret );
		M_EPILOG
	}
	int listxattr( char const* path_, char* buffer_, size_t size_ ) const {
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
					::memcpy( buffer_ + offset, s->c_str(), static_cast<int unsigned>( s->get_size() ) );
					buffer_[offset] = 0;
					offset += static_cast<int>( s->get_size() );
					++ offset;
				}
			} else if ( size_ > 0 ) {
				ret = -ERANGE;
			}
		}
		return ( ret );
		M_EPILOG
	}
	void utimens( char const* path_, struct timespec const time_[2] ) {
		M_PROLOG
		HLock l( _mutex );
		HXml::HNodeProxy n( get_node_by_path( path_ ) );
		HXml::HNode::properties_t& a( n.properties() );
		if ( time_[0].tv_nsec != UTIME_OMIT ) {
			if ( time_[0].tv_nsec == UTIME_NOW ) {
				a[ FILE::PROPERTY::TIME::MODIFICATION ] = now_local().to_string();
			} else {
				a[ FILE::PROPERTY::TIME::MODIFICATION ] = HTime( time_[0].tv_sec + HTime::SECONDS_TO_UNIX_EPOCH ).to_string();
			}
		}
		if ( time_[1].tv_nsec != UTIME_OMIT ) {
			if ( time_[1].tv_nsec == UTIME_NOW ) {
				a[ FILE::PROPERTY::TIME::ACCESS ] = now_local().to_string();
			} else {
				a[ FILE::PROPERTY::TIME::ACCESS ] = HTime( time_[1].tv_sec + HTime::SECONDS_TO_UNIX_EPOCH ).to_string();
			}
		}
		_synced = false;
		return;
		M_EPILOG
	}
	void unlink( char const* path_ ) {
		M_PROLOG
		HLock l( _mutex );
		HXml::HIterator it( get_node_it_by_path( path_ ) );
		HXml::HNodeProxy p( (*it).get_parent() );
		if ( ! have_access( p, W_OK | X_OK ) ) {
			throw HFileSystemException( "You have no permission to modify parent directory.", -EACCES );
		}
		p.remove_node( it );
		_synced = false;
		return;
		M_EPILOG
	}
	void flush( handle_t handle_ ) {
		M_PROLOG
		HLock l( _mutex );
		descriptor( handle_ ).flush();
		return;
		M_EPILOG
	}
	void lock( handle_t handle_, int cmd_, struct flock* lock_ ) {
		M_PROLOG
		HLock l( _mutex );
		descriptor( handle_ ).lock( cmd_, lock_ );
		return;
		M_EPILOG
	}
private:
	HFileSystem( HFileSystem const& ) = delete;
	HFileSystem& operator = ( HFileSystem const& ) = delete;
	handle_t create_handle( void ) {
		handle_t h( INVALID_HANDLE );
		if ( _availableDescriptors.is_empty() ) {
			h = _descriptorGenerator ++;
		} else {
			h = _availableDescriptors.top();
			_availableDescriptors.pop();
		}
		return ( h );
	}
	void release_handle( handle_t handle_ ) {
		M_PROLOG
		_availableDescriptors.push( handle_ );
		return;
		M_EPILOG
	}
	descriptors_t::iterator descriptor_iterator( handle_t handle_ ) {
		M_PROLOG
		descriptors_t::iterator it( _descriptors.find( handle_ ) );
		if ( ! ( it != _descriptors.end() ) ) {
			throw HFileSystemException( "Invalid handle: "_ys.append( handle_ ), -EBADF );
		}
		return ( it );
		M_EPILOG
	}
	HDescriptor& descriptor( handle_t handle_ ) {
		M_PROLOG
		descriptors_t::iterator it( _descriptors.find( handle_ ) );
		if ( ! ( it != _descriptors.end() ) ) {
			throw HFileSystemException( "Invalid handle: "_ys.append( handle_ ), -EBADF );
		}
		return ( it->second );
		M_EPILOG
	}
	HDescriptor const& descriptor( handle_t handle_ ) const {
		M_PROLOG
		descriptors_t::const_iterator it( _descriptors.find( handle_ ) );
		if ( ! ( it != _descriptors.end() ) ) {
			throw HFileSystemException( "Invalid handle: "_ys.append( handle_ ), -EBADF );
		}
		return ( it->second );
		M_EPILOG
	}
	bool have_access( HXml::HConstNodeProxy const& fsElem_, int mode_ ) const {
		mode_t mode( get_mode( fsElem_ ) );
		uid_t u( get_user( fsElem_ ) );
		gid_t g( get_group( fsElem_ ) );
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
		return ( ok );
	}
	HXml::HIterator get_node_it_by_path( tools::filesystem::path_t const& path_ ) {
		M_PROLOG
		typedef HArray<HString> components_t;
		components_t path( string::split<components_t>( filesystem::normalize_path( path_ ), "/", HTokenizer::SKIP_EMPTY ) );
		HXml::HNodeProxy n( _image.get_root() );
		HXml::HIterator nodeIt;
		for ( HString const& name : path ) {
			if ( ( name == "." ) || ( name == ".." ) ) {
				throw HFileSystemException( "Bogus path component: "_ys.append( path_ ).append( ": " ).append( name ), -EINVAL );
			}
			bool found( false );
			for ( HXml::HIterator it( n.begin() ), end( n.end() ); it != end; ++ it ) {
				if ( (*it).get_type() != HXml::HNode::TYPE::NODE ) {
					continue;
				}
				HXml::HNode::properties_t::const_iterator p( (*it).properties().find( FILE::PROPERTY::NAME ) );
				if ( ( p != (*it).properties().end() ) && ( p->second == name ) ) {
					nodeIt = it;
					n = *it;
					found = true;
					break;
				}
			}
			if ( ! found ) {
				throw HFileSystemException( "No such file or directory: "_ys.append( path_ ), -ENOENT );
			}
		}
		return ( nodeIt );
		M_EPILOG
	}
	HXml::HNodeProxy get_node_by_path( tools::filesystem::path_t const& path_ ) {
		M_PROLOG
		return ( path_ != filesystem::path::ROOT ? *get_node_it_by_path( path_ ) : _image.get_root() );
		M_EPILOG
	}
	HXml::HConstNodeProxy get_node_by_path( tools::filesystem::path_t const& path_ ) const {
		return ( const_cast<HFileSystem*>( this )->get_node_by_path( path_ ) );
	}
	static int get_hard_link_count( HXml::HConstNodeProxy const& n_ ) {
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
	void get_stat( HXml::HConstNodeProxy const& n_, struct stat* stat_ ) const {
		M_PROLOG
		::memset( stat_, 0, sizeof ( *stat_ ) );
		HXml::HNode::properties_t const& p( n_.properties() );
		HString type( n_.get_name() );
		stat_->st_dev = _devId;
		if ( type == FILE::TYPE::PLAIN ) {
			stat_->st_mode = S_IFREG;
			stat_->st_size = lexical_cast<i64_t>( p.at( FILE::PROPERTY::SIZE ) );
			stat_->st_nlink = 1;
		} else if ( type == FILE::TYPE::DIRECTORY ) {
			stat_->st_mode = S_IFDIR;
			stat_->st_nlink = static_cast<nlink_t>( get_hard_link_count( n_ ) );
			stat_->st_size = static_cast<off_t>( ( stat_->st_nlink ) * 160 );
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
		} else {
			throw HFileSystemException( "Bad file type: "_ys.append( type ) );
		}
		stat_->st_blksize = BLOCK_SIZE;
		stat_->st_blocks = ( stat_->st_size + BLOCK_SIZE - 1 ) / BLOCK_SIZE;

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
			log( LOG_LEVEL::ERROR ) << SYSCALL_FAILURE << endl;
			exit( 1 );
		}
		return ( curUmask );
	}
	mode_t get_mode( void ) const {
		mode_t const fullMode( S_IRUSR | S_IWUSR | S_IXUSR | S_IRGRP | S_IWGRP | S_IXGRP | S_IROTH | S_IWOTH | S_IXOTH );
		return ( fullMode & ~get_umask() );
	}
	HXml::HNodeProxy create_node( HXml::HNodeProxy& parent_, yaal::hcore::HString const& type_, yaal::hcore::HString const& name_, mode_t mode_ ) {
		M_PROLOG
		HXml::HNodeProxy n( *parent_.add_node( type_ ) );
		HXml::HNode::properties_t& a( n.properties() );
		a.insert( make_pair( FILE::PROPERTY::NAME, name_ ) );
		HString mode;
		mode.format( "%04o", ( mode_ & 0777 ) );
		a.insert( make_pair( FILE::PROPERTY::MODE, mode ) );
		a.insert( make_pair( FILE::PROPERTY::SIZE, HString( "0" ) ) );
		HString now( now_local().to_string() );
		a.insert( make_pair( FILE::PROPERTY::TIME::CHANGE, now ) );
		a.insert( make_pair( FILE::PROPERTY::TIME::MODIFICATION, now ) );
		a.insert( make_pair( FILE::PROPERTY::TIME::ACCESS, now ) );
		a.insert( make_pair( FILE::PROPERTY::USER, to_string( getuid() ) ) );
		a.insert( make_pair( FILE::PROPERTY::GROUP, to_string( getgid() ) ) );
		a.insert( make_pair( FILE::PROPERTY::INODE, to_string( _inodeGenerator ++ ) ) );
		return ( n );
		M_EPILOG
	}
	static bool is_plain( HXml::HConstNodeProxy const& node_ ) {
		return ( node_.get_name() == FILE::TYPE::PLAIN );
	}
	static bool is_directory( HXml::HConstNodeProxy const& node_ ) {
		return ( node_.get_name() == FILE::TYPE::PLAIN );
	}
};
yaal::hcore::HString const HFileSystem::ROOT_NODE( "root" );
yaal::hcore::HString const HFileSystem::FILE::PROPERTY::DEV_ID( "dev_id" );
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
yaal::hcore::HString const HFileSystem::FILE::TYPE::PLAIN( "file" );
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
		ret = e.code();
		log( LOG_LEVEL::ERROR ) << e.what() << ", " << -e.code() << endl;
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

int mkdir( char const* path_, mode_t mode_ ) {
	if ( setup._debug ) {
		log_trace << path_ << endl;
	}
	int ret( 0 );
	try {
		_fs_->mkdir( path_, mode_ );
	} catch ( HException const& e ) {
		ret = e.code();
		log( LOG_LEVEL::ERROR ) << e.what() << endl;
	}
	return ( ret );
}

int unlink( char const* path_ ) {
	if ( setup._debug ) {
		log_trace << path_ << endl;
	}
	int ret( 0 );
	try {
		_fs_->unlink( path_ );
	} catch ( HException const& e ) {
		ret = e.code();
		log( LOG_LEVEL::ERROR ) << e.what() << endl;
	}
	return ( ret );
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

int flush( char const* path_, struct fuse_file_info* info_ ) {
	if ( setup._debug ) {
		log_trace << path_ << endl;
	}
	int ret( 0 );
	try {
		_fs_->flush( info_->fh );
	} catch ( HException const& e ) {
		ret = e.code();
		log( LOG_LEVEL::ERROR ) << e.what() << endl;
	}
	return ( ret );
}

int release( char const* path_, struct fuse_file_info* info_ ) {
	if ( setup._debug ) {
		log_trace << path_ << endl;
	}
	int ret( 0 );
	try {
		_fs_->release( info_->fh );
	} catch ( HException const& e ) {
		ret = e.code();
		log( LOG_LEVEL::ERROR ) << e.what() << endl;
	}
	return ( ret );
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
		ret = e.code();
		log( LOG_LEVEL::ERROR ) << e.what() << endl;
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
		ret = e.code();
		log( LOG_LEVEL::ERROR ) << e.what() << endl;
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
		ret = e.code();
		log( LOG_LEVEL::ERROR ) << e.what() << endl;
	}
	return ( ret );
}

int readdir( char const* path_, void* buffer_, fuse_fill_dir_t filler_,
	off_t offset_, struct fuse_file_info* info_ ) {
	if ( setup._debug ) {
		log_trace << path_ << "(" << info_->fh << ")" << endl;
	}
	int ret( 0 );
	try {
		_fs_->readdir( buffer_, filler_, offset_, info_->fh );
	} catch ( HException const& e ) {
		ret = e.code();
		log( LOG_LEVEL::ERROR ) << e.what() << endl;
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
		ret = e.code();
		log( LOG_LEVEL::ERROR ) << e.what() << endl;
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
		log( LOG_LEVEL::ERROR ) << e.what() << endl;
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
		log( LOG_LEVEL::ERROR ) << e.what() << endl;
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
		ret = e.code();
		log( LOG_LEVEL::ERROR ) << e.what() << endl;
	}
	return ( ret );
}

int create( char const* path_, mode_t mode_, struct fuse_file_info* info_ ) {
	if ( setup._debug ) {
		log_trace << path_ << endl;
	}
	int ret( 0 );
	try {
		info_->fh = _fs_->create( path_, mode_ );
	} catch ( HException const& e ) {
		ret = e.code();
		log( LOG_LEVEL::ERROR ) << e.what() << endl;
	}
	return ( ret );
}

int ftruncate( char const*, off_t, struct fuse_file_info* ) {
	log << __PRETTY_FUNCTION__ << endl;
	return ( -1 );
}

int fgetattr( char const* path_, struct stat* stat_, struct fuse_file_info* info_ ) {
	if ( setup._debug ) {
		log_trace << path_ << endl;
	}
	int ret( 0 );
	try {
		_fs_->fgetattr( info_->fh, stat_ );
	} catch ( HException const& e ) {
		ret = e.code();
		log( LOG_LEVEL::ERROR ) << e.what() << ", " << -e.code() << endl;
	}
	return ( ret );
}

int lock( char const* path_, struct fuse_file_info* info_, int cmd_, struct flock* lock_ ) {
	if ( setup._debug ) {
		log_trace << path_ << endl;
	}
	int ret( 0 );
	try {
		_fs_->lock( info_->fh, cmd_, lock_ );
	} catch ( HException const& e ) {
		ret = e.code();
		log( LOG_LEVEL::ERROR ) << e.what() << ", " << -e.code() << endl;
	}
	return ( ret );
}

int utimens( char const* path_, struct timespec const time_[2] ) {
	if ( setup._debug ) {
		log_trace << path_ << endl;
	}
	int ret( 0 );
	try {
		_fs_->utimens( path_, time_ );
	} catch ( HException const& e ) {
		ret = e.code();
		log( LOG_LEVEL::ERROR ) << e.what() << ", " << -e.code() << endl;
	}
	return ( ret );
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

