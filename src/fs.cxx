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
#include <attr/xattr.h>

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
	static int const DIR_SIZE = 160;
	class HDescriptor final {
	public:
		enum class OPEN_MODE {
			READING,
			WRITING,
			READ_WRITE
		};
		enum class STATE {
			META,
			SYNC,
			DIRTY
		};
	private:
		tools::HXml::HNodeProxy _node;
		yaal::hcore::HChunk _data;
		int _size;
		OPEN_MODE _openMode;
		yaal::hcore::HChunk _encoded;
		pid_t _owner;
		int _openCount;
		STATE _state;
	public:
		HDescriptor( tools::HXml::HNodeProxy const& node_, OPEN_MODE openMode_ )
			: _node( node_ )
			, _data()
			, _size( 0 )
			, _openMode( openMode_ )
			, _encoded()
			, _owner( NO_OWNER )
			, _openCount( 0 )
			, _state( STATE::META ) {
			if ( is_plain( _node ) ) {
				_size = lexical_cast<int>( _node.properties().at( FILE::PROPERTY::SIZE ) );
			} else {
				M_ASSERT( is_directory( _node ) );
				_size = get_hard_link_count( _node ) * DIR_SIZE;
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
			if ( ! is_plain( _node ) ) {
				throw HFileSystemException( "File is not a plain file.", -EISDIR );
			}
			if ( _state == STATE::DIRTY ) {
				int oldSize( lexical_cast<int>( _node.properties().at( FILE::PROPERTY::SIZE ) ) );
				HXml::HNodeProxy value;
				if ( oldSize > 0 ) {
					/* Data existed. */
					HXml::HIterator valueIt = content();
					if ( _size > 0 ) {
						/* Data will continue to exist. */
						value = *valueIt;
					} else {
						/* Data shall be removed. */
						_node.remove_node( valueIt );
					}
				} else {
					/* Data did not exist yet. */
					if ( _size > 0 ) {
						/* Data should be created. */
						value = *(_node.add_node( HXml::HNode::TYPE::CONTENT ) );
					}
				}
				if ( _size > 0 ) {
					HMemoryObserver srcMem( _data.raw(), _size );
					HMemoryProvider dstMem( _encoded, 0 );
					HMemory src( srcMem );
					HMemory dst( dstMem );
					base64::encode( src, dst );
					HString data( static_cast<char*>( dstMem.get_memory() ), dstMem.get_size() );
					value.set_value( data );
				}
				_node.properties()[ FILE::PROPERTY::SIZE ] = _size;
				_state = STATE::SYNC;
			}
			return;
			M_EPILOG
		}
		void inc_open_count( void ) {
			++ _openCount;
		}
		void dec_open_count( void ) {
			-- _openCount;
		}
		bool is_opened( void ) const {
			return ( _openCount > 0 );
		}
		void lock( int cmd_, struct flock* lock_ ) {
			M_PROLOG
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
			return;
			M_EPILOG
		}
		void truncate( int size_ ) {
			M_PROLOG
			if ( size_ < 0 ) {
				throw HFileSystemException( "Bad size.", -EINVAL );
			}
			if ( ! is_plain( _node ) ) {
				throw HFileSystemException( "File is not a plain file.", -EISDIR );
			}
			if ( _state == STATE::META ) {
				int metaSize( lexical_cast<int>( _node.properties().at( FILE::PROPERTY::SIZE ) ) );
				if ( metaSize != size_ ) {
					decode();
				}
			}
			if ( size_ != _size ) {
				_state = STATE::DIRTY;
				if ( size_ > 0 ) {
					_data.realloc( size_ );
				}
				_size = size_;
			}
			return;
			M_EPILOG
		}
		int write_buf( fuse_bufvec const* buf_, int offset_ ) {
			M_PROLOG
			if ( offset_ < 0 ) {
				throw HFileSystemException( "Bad offset.", -EINVAL );
			}
			if ( ! is_plain( _node ) ) {
				throw HFileSystemException( "File is not a plain file.", -EISDIR );
			}
			int totalSize( static_cast<int>( fuse_buf_size( buf_ ) ) );
			if ( _state == STATE::META ) {
				if ( _size > 0 ) {
					decode();
				}
			}
			int newSize( max( _size, offset_ + totalSize ) );
			_data.realloc( newSize );
			fuse_bufvec dst;
			::memset( &dst, 0, sizeof ( dst ) );
			dst.buf[0].mem = _data.get<char>() + offset_;
			dst.buf[0].size = static_cast<size_t>( totalSize );
			dst.count = 1;
			int written( static_cast<int>( fuse_buf_copy( &dst, const_cast<fuse_bufvec*>( buf_ ), FUSE_BUF_NO_SPLICE ) ) );
			_size = newSize;
			_state = STATE::DIRTY;
			return ( written );
			M_EPILOG
		}
		int write( char const* buf_, int size_, int offset_ ) {
			M_PROLOG
			if ( size_ < 0 ) {
				throw HFileSystemException( "Bad offset.", -EINVAL );
			}
			if ( offset_ < 0 ) {
				throw HFileSystemException( "Bad offset.", -EINVAL );
			}
			if ( ! is_plain( _node ) ) {
				throw HFileSystemException( "File is not a plain file.", -EISDIR );
			}
			if ( _state == STATE::META ) {
				if ( _size > 0 ) {
					decode();
				}
			}
			int newSize( max( _size, offset_ + size_ ) );
			_data.realloc( newSize );
			::memcpy( _data.get<char>() + offset_, buf_, static_cast<size_t>( size_ ) );
			_size = newSize;
			_state = STATE::DIRTY;
			return ( _size );
			M_EPILOG
		}
		int read_buf( fuse_bufvec** dst_, int size_, int offset_ ) {
			M_PROLOG
			if ( offset_ < 0 ) {
				throw HFileSystemException( "Bad offset.", -EINVAL );
			}
			if ( size_ < 0 ) {
				throw HFileSystemException( "Bad size.", -EINVAL );
			}
			if ( ! is_plain( _node ) ) {
				throw HFileSystemException( "File is not a plain file.", -EISDIR );
			}
			if ( _state == STATE::META ) {
				if ( _size > 0 ) {
					decode();
				}
			}
			int toRead( min( max( _size - offset_, 0 ), size_ ) );
			*dst_ = memory::calloc<fuse_bufvec>( 1 );
			if ( toRead > 0 ) {
				(*dst_)->buf[0].mem = memory::alloc( toRead );
				(*dst_)->buf[0].size = static_cast<size_t>( toRead );
				(*dst_)->count = 1;
				::memcpy( (*dst_)->buf[0].mem, _data.get<char const>() + offset_, static_cast<size_t>( toRead ) );
			}
			return ( toRead );
			M_EPILOG
		}
		int read( char* buffer_, int size_, int offset_ ) {
			M_PROLOG
			if ( offset_ < 0 ) {
				throw HFileSystemException( "Bad offset.", -EINVAL );
			}
			if ( size_ < 0 ) {
				throw HFileSystemException( "Bad size.", -EINVAL );
			}
			if ( ! is_plain( _node ) ) {
				throw HFileSystemException( "File is not a plain file.", -EISDIR );
			}
			if ( _state == STATE::META ) {
				if ( _size > 0 ) {
					decode();
				}
			}
			int toRead( min( max( _size - offset_, 0 ), size_ ) );
			::memcpy( buffer_, _data.get<char const>() + offset_, static_cast<size_t>( toRead ) );
			return ( toRead );
			M_EPILOG
		}
		bool is_flushed( void ) const {
			return ( _state != STATE::DIRTY );
		}
	private:
		void decode( void ) {
			M_PROLOG
			M_ASSERT( _state == STATE::META );
			if ( _size > 0 ) {
				HXml::HNodeProxy value( *content() );
				HString const& data( value.get_value() );
				int skipStart( static_cast<int>( data.find_other_than( _whiteSpace_.data() ) ) );
				int skipEnd( static_cast<int>( data.reverse_find_other_than( _whiteSpace_.data() ) ) );
				int toRead( static_cast<int>( data.get_size() ) - ( skipStart + skipEnd ) );
//				log_trace << "skipStart = " << skipStart << ", skipEnd = " << skipEnd << ", toRead = " << toRead << endl;
				if ( toRead > 0 ) {
					HMemoryObserver srcMem( const_cast<char*>( data.raw() ) + skipStart, toRead );
					HMemoryProvider dstMem( _data, 0 );
					HMemory src( srcMem );
					HMemory dst( dstMem );
					base64::decode( src, dst );
					int decodedSize( static_cast<int>( dstMem.get_size() ) );
					if ( decodedSize == _size ) {
						_state = STATE::SYNC;
					} else {
						log_trace << "Data corruption, expected " << _size << " bytes, got " << decodedSize << endl;
						_size = decodedSize;
						_state = STATE::DIRTY;
					}
				} else {
					M_ASSERT( toRead == 0 );
					_size = 0;
					_state = STATE::DIRTY;
					log_trace << "Data corruption, no data has been found when it was expected." << endl;
				}
			}
			return;
			M_EPILOG
		}
		HXml::HIterator content( void ) {
			M_PROLOG
			HXml::HIterator it( _node.begin() );
			while ( it != _node.end() ) {
				if ( (*it).get_type() == HXml::HNode::TYPE::CONTENT ) {
					break;
				}
				++ it;
			}
			return ( it );
			M_EPILOG
		}
		HDescriptor( HDescriptor const& ) = delete;
		HDescriptor& operator = ( HDescriptor const& ) = delete;
	};
private:
	yaal::tools::filesystem::path_t _imagePath;
	yaal::tools::HXml _image;
	dev_t _devId;
	typedef yaal::u64_t handle_t;
	typedef yaal::hcore::HHashMap<handle_t, u64_t> inodes_t;
	typedef yaal::hcore::HHashMap<u64_t, HDescriptor> descriptors_t;
	typedef yaal::hcore::HStack<handle_t> available_descriptors_t;
	inodes_t _inodes;
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
			static yaal::hcore::HString const TARGET;
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
		, _devId( static_cast<dev_t>( hash<yaal::hcore::HString>()( imagePath_ ) ) )
		, _inodes()
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
			p.insert( make_pair( FILE::PROPERTY::INODE, to_string( _inodeGenerator ++ ) ) );
			p.insert( make_pair( FILE::PROPERTY::INODE_NEXT, to_string( _inodeGenerator.load() ) ) );
			_synced = false;
		}
		return;
		M_EPILOG
	}
	~HFileSystem( void ) {
		M_PROLOG
		if ( ! ( _inodes.is_empty() && _descriptors.is_empty() ) ) {
			log( LOG_LEVEL::ERROR )
				<< "descriptor leak! (inodes=" << _inodes.get_size()
				<< ",descriptors=" << _descriptors.get_size() << ")" << endl;
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
		HXml::HNodeProxy n( get_node_by_path( path_ ) );
		if ( n.get_name() != FILE::TYPE::DIRECTORY ) {
			throw HFileSystemException( "Path does not point to a directory: "_ys.append( path_ ), -ENOTDIR );
		}
		handle_t h( create_handle() );
		u64_t inode( get_inode( n ) );
		_inodes.insert( make_pair( h, inode ) );
		descriptors_t::iterator descIt( _descriptors.insert( descriptors_t::value_type( inode, HDescriptor( n, HDescriptor::OPEN_MODE::READING ) ) ).first );
		descIt->second.inc_open_count();
		return ( h );
		M_EPILOG
	}
	void releasedir( handle_t handle_ ) {
		M_PROLOG
		release( handle_ );
		return;
		M_EPILOG
	}
	handle_t open( tools::filesystem::path_t const& path_, int flags_ ) {
		M_PROLOG
		HLock l( _mutex );
		HXml::HNodeProxy n( get_node_by_path( path_ ) );
		if ( n.get_name() != FILE::TYPE::PLAIN ) {
			throw HFileSystemException( "Path does not point to a plain file: "_ys.append( path_ ), -EISDIR );
		}
		handle_t h( open_handle( n, flags_ ) );
		_synced = false;
		return ( h );
		M_EPILOG
	}
	void release( handle_t handle_ ) {
		M_PROLOG
		HLock l( _mutex );
		inodes_t::iterator inodeIt( _inodes.find( handle_ ) );
		if ( ! ( inodeIt != _inodes.end() ) ) {
			throw HFileSystemException( "Invalid handle: "_ys.append( handle_ ), -EBADF );
		}
		descriptors_t::iterator descIt( _descriptors.find( inodeIt->second ) );
		M_ASSERT( descIt != _descriptors.end() );
		descIt->second.dec_open_count();
		if ( ! descIt->second.is_opened() ) {
			_descriptors.erase( descIt );
		}
		_inodes.erase( inodeIt );
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
			if ( c.get_name() != FILE::CONTENT::XATTR ) {
				HString const& name( c.properties().at( FILE::PROPERTY::NAME ) );
				get_stat( c, &s );
				if ( filler_( buf_, name.c_str(), &s, 0 ) ) {
					throw HFileSystemException( "Directory buffer is full." );
				}
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
	handle_t create( tools::filesystem::path_t const& path_, int flags_, mode_t mode_ ) {
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
		handle_t h( open_handle( n, flags_ ) );
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
			if ( ( c.get_type() == HXml::HNode::TYPE::NODE ) && ( c.get_name() == FILE::CONTENT::XATTR ) ) {
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
			HString value( base64::decode( (*n.begin()).get_value() ) );
			ret = static_cast<int>( value.get_size() );
			if ( ret <= static_cast<int>( size_ ) ) {
				::memcpy( buffer_, value.c_str(), static_cast<int unsigned>( ret ) );
			} else if ( static_cast<int>( size_ ) > 0 ) {
				throw HFileSystemException( "Buffer for extended attributes is too small.", -ERANGE );
			}
		} else {
			ret = -ENODATA;
		}
		return ( ret );
		M_EPILOG
	}
	void setxattr( char const* path_, HString const& name_, char const* buffer_, int size_, int flags_ ) {
		M_PROLOG
		HLock l( _mutex );
		HXml::HNodeProxy n( get_node_by_path( path_ ) );
		int pos( static_cast<int>( name_.find( '.' ) ) );
		if ( pos == HString::npos ) {
			throw HFileSystemException( "Invalid attribute name: "_ys.append( name_ ), -EINVAL );
		}
		bool ok( false );
		for ( HXml::HNodeProxy c : n ) {
			if ( ( c.get_type() == HXml::HNode::TYPE::NODE ) && ( c.get_name() == FILE::CONTENT::XATTR ) ) {
				n = c;
				ok = true;
				break;
			}
		}
		if ( ! ok ) {
			/* Extended attributes did not exist yet. */
			n = *n.add_node( FILE::CONTENT::XATTR );
		}
		ok = false;
		HString ns( name_.left( pos ) );
		for ( HXml::HNodeProxy c : n ) {
			if ( c.get_name() == ns ) {
				ok = true;
				n = c;
				break;
			}
		}
		if ( ! ok ) {
			/* Given namespace does not exist yet. */
			n = *n.add_node( ns );
		}
		ok = false;
		HString name( name_.mid( pos + 1 ) );
		for ( HXml::HNodeProxy c : n ) {
			if ( c.get_name() == name ) {
				ok = true;
				n = c;
				break;
			}
		}
		if ( ok && ( flags_ == XATTR_CREATE ) ) {
			throw HFileSystemException( "ExAttr already exists: "_ys.append( name ), -EEXIST );
		} else if ( ! ok && ( flags_ == XATTR_REPLACE ) ) {
			throw HFileSystemException( "ExAttr does not exist: "_ys.append( name ), -ENOATTR );
		}
		if ( ! ok ) {
			n = *n.add_node( name );
			n = *n.add_node( HXml::HNode::TYPE::CONTENT );
		} else if ( n.has_childs() ) {
			n = *n.begin();
		} else {
			n = *n.add_node( HXml::HNode::TYPE::CONTENT );
		}
		HString value( buffer_, size_ );
		n.set_value( base64::encode( value ) );
		_synced = false;
		return;
		M_EPILOG
	}
	int listxattr( char const* path_, char* buffer_, int size_ ) const {
		M_PROLOG
		HLock l( _mutex );
		HXml::HConstNodeProxy n( get_node_by_path( path_ ) );
		bool ok( false );
		for ( HXml::HConstNodeProxy c : n ) {
			if ( ( c.get_type() == HXml::HNode::TYPE::NODE ) && ( c.get_name() == FILE::CONTENT::XATTR ) ) {
				n = c;
				ok = true;
				break;
			}
		}
		int size( 0 );
		if ( ok ) {
			for ( HXml::HConstNodeProxy ns : n ) {
				HString const& nsn( ns.get_name() );
				int nss( static_cast<int>( nsn.get_size() ) );
				for ( HXml::HConstNodeProxy attr : ns ) {
					if ( size_ > 0 ) {
						if ( ( size + nss + 1 ) > size_ ) {
							throw HFileSystemException( "Buffer too small (ns).", -ERANGE );
						}
						::memcpy( buffer_ + size, nsn.c_str(), static_cast<int unsigned>( nss ) );
					}
					size += nss;
					if ( size_ > 0 ) {
						buffer_[size] = '.';
					}
					++ size;

					HString const& an( attr.get_name() );
					int as( static_cast<int>( an.get_size() ) );
					if ( size_ > 0 ) {
						if ( ( size + as + 1 ) > size_ ) {
							throw HFileSystemException( "Buffer too small.", -ERANGE );
						}
						::memcpy( buffer_ + size, an.c_str(), static_cast<int unsigned>( as ) );
					}
					size += as;
					if ( size_ > 0 ) {
						buffer_[size] = 0;
					}
					++ size;
				}
			}
		}
		return ( size );
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
	void rmdir( char const* path_ ) {
		M_PROLOG
		unlink( path_ );
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
	void truncate( char const* path_, int size_ ) {
		M_PROLOG
		HLock l( _mutex );
		HXml::HNodeProxy n( get_node_by_path( path_ ) );
		u64_t inode( get_inode( n ) );
		descriptors_t::iterator it( _descriptors.find( inode ) );
		if ( it != _descriptors.end() ) {
			it->second.truncate( size_ );
		} else {
			HDescriptor d( n, HDescriptor::OPEN_MODE::WRITING );
			d.truncate( size_ );
			d.flush();
		}
		_synced = false;
		return;
		M_EPILOG
	}
	void ftruncate( handle_t handle_, int size_ ) {
		M_PROLOG
		HLock l( _mutex );
		descriptor( handle_ ).truncate( size_ );
		_synced = false;
		return;
		M_EPILOG
	}
	int write_buf( handle_t handle_, fuse_bufvec const* buf_, int offset_ ) {
		M_PROLOG
		HLock l( _mutex );
		int ret( descriptor( handle_ ).write_buf( buf_, offset_ ) );
		_synced = false;
		return ( ret );
		M_EPILOG
	}
	int write( handle_t handle_, char const* buf_, int size_, int offset_ ) {
		M_PROLOG
		HLock l( _mutex );
		int ret( descriptor( handle_ ).write( buf_, size_, offset_ ) );
		_synced = false;
		return ( ret );
		M_EPILOG
	}
	int read_buf( handle_t handle_, fuse_bufvec** buf_, int size_, int offset_ ) {
		M_PROLOG
		HLock l( _mutex );
		int ret( descriptor( handle_ ).read_buf( buf_, size_, offset_ ) );
		return ( ret );
		M_EPILOG
	}
	int read( handle_t handle_, char* buf_, int size_, int offset_ ) {
		M_PROLOG
		HLock l( _mutex );
		int ret( descriptor( handle_ ).read( buf_, size_, offset_ ) );
		return ( ret );
		M_EPILOG
	}
	void chmod( char const* path_, mode_t mode_ ) {
		M_PROLOG
		HLock l( _mutex );
		HXml::HNodeProxy n( get_node_by_path( path_ ) );
		set_mode( n, mode_ );
		_synced = false;
		return;
		M_EPILOG
	}
	void chown( char const* path_, uid_t uid_, gid_t gid_ ) {
		M_PROLOG
		HLock l( _mutex );
		HXml::HNodeProxy n( get_node_by_path( path_ ) );
		HXml::HNode::properties_t& a( n.properties() );
		a[ FILE::PROPERTY::USER ] = to_string( uid_ );
		a[ FILE::PROPERTY::GROUP ] = to_string( gid_ );
		_synced = false;
		return;
		M_EPILOG
	}
	void rename( filesystem::path_t const& from_, filesystem::path_t const& to_ ) {
		M_PROLOG
		HLock l( _mutex );
		path_t fromDName( dirname( from_ ) );
		path_t toDName( dirname( to_ ) );
		path_t toBName( basename( to_ ) );
		HXml::HIterator fromIt( get_node_it_by_path( from_ ) );
		HXml::HNodeProxy fromDir( (*fromIt).get_parent() );
		HXml::HNodeProxy toDir( get_node_by_path( toDName ) );
		HXml::HIterator toIt( toDir.end() );
		try {
			toIt = get_node_it_by_path( to_ );
		} catch ( ... ) {
		}
		if ( ! have_access( fromDir, W_OK | X_OK ) ) {
			throw HFileSystemException( "You have no permission to modify parent directory.", -EACCES );
		}
		if ( ! is_directory( fromDir ) ) {
			throw HFileSystemException( "Path does not point to a directory: "_ys.append( fromDName ), -ENOTDIR );
		}
		bool srcDir( is_directory( *fromIt ) );
		if ( toIt != toDir.end() ) {
			/* Destination already exists. */
			if ( srcDir ) {
				/* Source is a directory. */
				if ( ! is_directory( *toIt ) ) {
					throw HFileSystemException( "Path does not point to a directory: "_ys.append( to_ ), -ENOTDIR );
				}
				if ( ! (*toIt).disjointed( *fromIt ) ) {
					throw HFileSystemException( "The new pathname contained a path prefix of the old.", -EINVAL );
				}
				if ( ! have_access( *toIt, W_OK | X_OK ) ) {
					throw HFileSystemException( "You have no permission to modify destination directory.", -EACCES );
				}
				(*toIt).move_node( *fromIt );
			} else {
				/* Source is a regular file. */
				if ( is_directory( *toIt ) ) {
					if ( ! (*toIt).disjointed( *fromIt ) ) {
						throw HFileSystemException( "The new pathname contained a path prefix of the old.", -EINVAL );
					}
					if ( ! have_access( *toIt, W_OK | X_OK ) ) {
						throw HFileSystemException( "You have no permission to modify destination directory.", -EACCES );
					}
					(*toIt).move_node( *fromIt );
				} else {
					/* Both source and destination exist and both are regular files. */
					if ( ! (*toIt).disjointed( *fromIt ) ) {
						throw HFileSystemException( "The new pathname contained a path prefix of the old.", -EINVAL );
					}
					if ( ! have_access( toDir, W_OK | X_OK ) ) {
						throw HFileSystemException( "You have no permission to modify destination directory.", -EACCES );
					}
					toDir.replace_node( toIt, *fromIt );
				}
			}
		} else {
			/* Destination does not exist. */
			if ( ! is_directory( toDir ) ) {
				throw HFileSystemException( "Path does not point to a directory: "_ys.append( toDName ), -ENOTDIR );
			}
			if ( ! toDir.disjointed( *fromIt ) ) {
				throw HFileSystemException( "The new pathname contained a path prefix of the old.", -EINVAL );
			}
			if ( ! have_access( toDir, W_OK | X_OK ) ) {
				throw HFileSystemException( "You have no permission to modify destination directory.", -EACCES );
			}
			HXml::HNodeProxy node( *fromIt );
			toDir.move_node( *fromIt );
			node.properties()[FILE::PROPERTY::NAME] = toBName;
		}
		_synced = false;
		return;
		M_EPILOG
	}
	void symlink( filesystem::path_t const& target_, filesystem::path_t const& link_ ) {
		M_PROLOG
		HLock l( _mutex );
		path_t linkDName( dirname( link_ ) );
		HXml::HNodeProxy linkBaseDir( get_node_by_path( linkDName ) );
		try {
			get_node_it_by_path( link_ );
		} catch ( HFileSystemException const& e ) {
			if ( e.code() != -ENOENT ) {
				throw HFileSystemException( "Link path already exists: "_ys.append( link_ ), -EEXIST );
			}
		}
		HXml::HNodeProxy link( create_node( linkBaseDir, FILE::TYPE::SYMLINK, basename( link_ ), get_mode() ) );
		HXml::HNode::properties_t& a( link.properties() );
		a[FILE::PROPERTY::TARGET] = target_;
		_synced = false;
		return;
		M_EPILOG
	}
	void readlink( char const* path_, char* buffer_, int size_ ) const {
		M_PROLOG
		HLock l( _mutex );
		if ( size_ <= 0 ) {
			throw HFileSystemException( "Buffer size not positive: "_ys.append( -EINVAL ) );
		}
		HXml::HConstNodeProxy link( get_node_by_path( path_ ) );
		if ( ! is_symlink( link ) ) {
			throw HFileSystemException( "Not a symlink:"_ys.append( path_ ), -EINVAL );
		}
		HString const& target( link.properties().at( FILE::PROPERTY::TARGET ) );
		int len( min( static_cast<int>( target.get_length() ), size_ - 1 ) );
		::strncpy( buffer_, target.raw(), static_cast<size_t>( len ) );
		buffer_[len] = 0;
		if ( len < target.get_length() ) {
			throw HFileSystemException( "", -ENAMETOOLONG );
		}
		return;
		M_EPILOG
	}
	void fsync( handle_t handle_ ) {
		M_PROLOG
		HLock l( _mutex );
		descriptor( handle_ ).flush();
		save();
		for ( descriptors_t::value_type const& di : _descriptors ) {
			if ( ! di.second.is_flushed() ) {
				_synced = false;
				break;
			}
		}
		return;
		M_EPILOG
	}
	void fsyncdir( handle_t handle_ ) {
		M_PROLOG
		HLock l( _mutex );
		inodes_t::iterator inodeIt( _inodes.find( handle_ ) );
		if ( ! ( inodeIt != _inodes.end() ) ) {
			throw HFileSystemException( "Invalid handle: "_ys.append( handle_ ), -EBADF );
		}
		u64_t inode( inodeIt->second );
		for ( descriptors_t::value_type& di : _descriptors ) {
			HXml::HNodeProxy p( di.second.node() );
			if ( ! is_plain( p ) ) {
				continue;
			}
			while ( !! p ) {
				if ( p.properties().at( FILE::PROPERTY::INODE ) == inode ) {
					di.second.flush();
					break;
				}
				p = p.get_parent();
			}
		}
		save();
		for ( descriptors_t::value_type const& di : _descriptors ) {
			if ( ! di.second.is_flushed() ) {
				_synced = false;
				break;
			}
		}
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
	HDescriptor& descriptor( handle_t handle_ ) {
		M_PROLOG
		inodes_t::iterator inodeIt( _inodes.find( handle_ ) );
		if ( ! ( inodeIt != _inodes.end() ) ) {
			throw HFileSystemException( "Invalid handle: "_ys.append( handle_ ), -EBADF );
		}
		descriptors_t::iterator descIt( _descriptors.find( inodeIt->second ) );
		M_ASSERT( descIt != _descriptors.end() );
		return ( descIt->second );
		M_EPILOG
	}
	HDescriptor const& descriptor( handle_t handle_ ) const {
		M_PROLOG
		inodes_t::const_iterator inodeIt( _inodes.find( handle_ ) );
		if ( ! ( inodeIt != _inodes.end() ) ) {
			throw HFileSystemException( "Invalid handle: "_ys.append( handle_ ), -EBADF );
		}
		descriptors_t::const_iterator descIt( _descriptors.find( inodeIt->second ) );
		M_ASSERT( descIt != _descriptors.end() );
		return ( descIt->second );
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
		bool haveAccess( true );
		for ( HString const& name : path ) {
			if ( ( name == "." ) || ( name == ".." ) ) {
				throw HFileSystemException( "Bogus path component: "_ys.append( path_ ).append( ": " ).append( name ), -EINVAL );
			}
			if ( ! haveAccess ) {
				throw HFileSystemException( "Access denied.", -EACCES );
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
					haveAccess = have_access( n, X_OK );
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
			if ( is_directory( c ) ) {
				++ hlc;
			}
		}
		return ( hlc );
	}
	u64_t get_inode( HXml::HConstNodeProxy const& n_ ) const {
		M_PROLOG
		return ( lexical_cast<uid_t>( n_.properties().at( FILE::PROPERTY::INODE ) ) );
		M_EPILOG
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
			stat_->st_size = static_cast<off_t>( ( stat_->st_nlink ) * DIR_SIZE );
		} else if ( type == FILE::TYPE::SYMLINK ) {
			stat_->st_mode = S_IFLNK;
			stat_->st_size = p.at( FILE::PROPERTY::TARGET ).get_length();
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
		a.insert( make_pair( FILE::PROPERTY::SIZE, HString( "0" ) ) );
		HString now( now_local().to_string() );
		a.insert( make_pair( FILE::PROPERTY::TIME::CHANGE, now ) );
		a.insert( make_pair( FILE::PROPERTY::TIME::MODIFICATION, now ) );
		a.insert( make_pair( FILE::PROPERTY::TIME::ACCESS, now ) );
		a.insert( make_pair( FILE::PROPERTY::USER, to_string( getuid() ) ) );
		a.insert( make_pair( FILE::PROPERTY::GROUP, to_string( getgid() ) ) );
		a.insert( make_pair( FILE::PROPERTY::INODE, to_string( _inodeGenerator ++ ) ) );
		set_mode( n, mode_ );
		return ( n );
		M_EPILOG
	}
	void set_mode( HXml::HNodeProxy& node_, mode_t mode_ ) {
		M_PROLOG
		node_.properties()[ FILE::PROPERTY::MODE ].format( "%04o", mode_ & 07777 );
		M_EPILOG
	}
	handle_t open_handle( HXml::HNodeProxy& node_, int flags_ ) {
		M_PROLOG
		HDescriptor::OPEN_MODE mode( HDescriptor::OPEN_MODE::READING );
		if ( flags_ & O_RDWR ) {
			mode = HDescriptor::OPEN_MODE::READ_WRITE;
		} else if ( flags_ & O_WRONLY ) {
			mode = HDescriptor::OPEN_MODE::WRITING;
		}
		handle_t h( create_handle() );
		u64_t inode( get_inode( node_ ) );
		_inodes.insert( make_pair( h, inode ) );
		descriptors_t::iterator descIt( _descriptors.insert( descriptors_t::value_type( inode, HDescriptor( node_, mode ) ) ).first );
		descIt->second.inc_open_count();
		return ( h );
		M_EPILOG
	}
	static bool is_plain( HXml::HConstNodeProxy const& node_ ) {
		return ( node_.get_name() == FILE::TYPE::PLAIN );
	}
	static bool is_directory( HXml::HConstNodeProxy const& node_ ) {
		return ( node_.get_name() == FILE::TYPE::DIRECTORY );
	}
	static bool is_symlink( HXml::HConstNodeProxy const& node_ ) {
		return ( node_.get_name() == FILE::TYPE::SYMLINK );
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
yaal::hcore::HString const HFileSystem::FILE::PROPERTY::TARGET( "target" );
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

int readlink( char const* path_, char* buffer_, size_t size_ ) {
	if ( setup._debug ) {
		log_trace << path_ << endl;
	}
	int ret( 0 );
	try {
		_fs_->readlink( path_, buffer_, static_cast<int>( size_ ) );
	} catch ( HException const& e ) {
		ret = e.code();
		log( LOG_LEVEL::ERROR ) << e.what() << ", " << -e.code() << endl;
	}
	return ( ret );
}

int open( char const* path_, struct fuse_file_info* info_ ) {
	if ( setup._debug ) {
		log_trace << path_ << ", flags: " << info_->flags << endl;
	}
	int ret( 0 );
	try {
		info_->fh = _fs_->open( path_, info_->flags );
	} catch ( HException const& e ) {
		ret = e.code();
		log( LOG_LEVEL::ERROR ) << e.what() << endl;
	}
	return ( ret );
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

int rmdir( char const* path_ ) {
	if ( setup._debug ) {
		log_trace << path_ << endl;
	}
	int ret( 0 );
	try {
		_fs_->rmdir( path_ );
	} catch ( HException const& e ) {
		ret = e.code();
		log( LOG_LEVEL::ERROR ) << e.what() << endl;
	}
	return ( ret );
}

int symlink( char const* target_, char const* link_ ) {
	if ( setup._debug ) {
		log_trace << link_ << "->" << target_ << endl;
	}
	int ret( 0 );
	try {
		_fs_->symlink( target_, link_ );
	} catch ( HException const& e ) {
		ret = e.code();
		log( LOG_LEVEL::ERROR ) << e.what() << endl;
	}
	return ( ret );
}

int rename( char const* from_, char const* to_ ) {
	if ( setup._debug ) {
		log_trace << from_ << "->" << to_ << endl;
	}
	int ret( 0 );
	try {
		_fs_->rename( from_, to_ );
	} catch ( HException const& e ) {
		ret = e.code();
		log( LOG_LEVEL::ERROR ) << e.what() << endl;
	}
	return ( ret );
}

int link( char const*, char const* ) {
	log << __PRETTY_FUNCTION__ << endl;
	return ( -EINVAL );
}

int chmod( char const* path_, mode_t mode_ ) {
	if ( setup._debug ) {
		log_trace << path_ << endl;
	}
	int ret( 0 );
	try {
		_fs_->chmod( path_, mode_ );
	} catch ( HException const& e ) {
		ret = e.code();
		log( LOG_LEVEL::ERROR ) << e.what() << endl;
	}
	return ( ret );
}

int chown( char const* path_, uid_t uid_, gid_t gid_ ) {
	if ( setup._debug ) {
		log_trace << path_ << endl;
	}
	int ret( 0 );
	try {
		_fs_->chown( path_, uid_, gid_ );
	} catch ( HException const& e ) {
		ret = e.code();
		log( LOG_LEVEL::ERROR ) << e.what() << endl;
	}
	return ( ret );
}

int truncate( char const* path_, off_t size_ ) {
	if ( setup._debug ) {
		log_trace << path_ << ", " << size_ << endl;
	}
	int ret( 0 );
	try {
		_fs_->truncate( path_, static_cast<int>( size_ ) );
	} catch ( HException const& e ) {
		ret = e.code();
		log( LOG_LEVEL::ERROR ) << e.what() << endl;
	}
	return ( ret );
}

int read( char const* path_, char* buffer_, size_t size_, off_t offset_,
	struct fuse_file_info* info_ ) {
	if ( setup._debug ) {
		log_trace << path_ << ", to read: " << size_ << ", at offset: " << offset_ << endl;
	}
	int ret( 0 );
	try {
		ret = _fs_->read( info_->fh, buffer_, static_cast<int>( size_ ), static_cast<int>( offset_ ) );
		if ( setup._debug ) {
			log_trace << path_ << ", read: " << ret << endl;
		}
	} catch ( HException const& e ) {
		ret = e.code();
		log( LOG_LEVEL::ERROR ) << e.what() << ", " << -e.code() << endl;
	}
	return ( ret );
}

int write( char const* path_, char const* buffer_, size_t size_, off_t offset_, struct fuse_file_info* info_ ) {
	if ( setup._debug ) {
		log_trace << path_ << ", to write: " << size_ << endl;
	}
	int ret( 0 );
	try {
		ret = _fs_->write( info_->fh, buffer_, static_cast<int>( size_ ), static_cast<int>( offset_ ) );
		if ( setup._debug ) {
			log_trace << path_ << ", written: " << ret << endl;
		}
	} catch ( HException const& e ) {
		ret = e.code();
		log( LOG_LEVEL::ERROR ) << e.what() << ", " << -e.code() << endl;
	}
	return ( ret );
}

int statfs( char const*, struct statvfs * ) {
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

int fsync( char const* path_, int, struct fuse_file_info* info_ ) {
	if ( setup._debug ) {
		log_trace << path_ << endl;
	}
	int ret( 0 );
	try {
		_fs_->fsync( info_->fh );
	} catch ( HException const& e ) {
		ret = e.code();
		log( LOG_LEVEL::ERROR ) << e.what() << endl;
	}
	return ( ret );
}

int setxattr( char const* path_, char const* name_, char const* value_, size_t size_, int flags_ ) {
	if ( setup._debug ) {
		log_trace << path_ << endl;
	}
	int ret( 0 );
	try {
		_fs_->setxattr( path_, name_, value_, static_cast<int>( size_ ), flags_ );
	} catch ( HException const& e ) {
		ret = e.code();
		log( LOG_LEVEL::ERROR ) << e.what() << endl;
	}
	return ( ret );
}

int getxattr( char const* path_, char const* name_, char* buffer_, size_t size_ ) {
	if ( setup._debug ) {
		log_trace << path_ << ", for name: " << name_ <<  endl;
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
		log_trace << path_ << ", buffer size = " << size_ <<  endl;
	}
	int ret( 0 );
	try {
		ret = _fs_->listxattr( path_, buffer_, static_cast<int>( size_ ) );
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
		log_trace << path_ << " (" << info_->fh << ")" << endl;
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

int fsyncdir( char const* path_, int, struct fuse_file_info* info_ ) {
	if ( setup._debug ) {
		log_trace << path_ << endl;
	}
	int ret( 0 );
	try {
		_fs_->fsyncdir( info_->fh );
	} catch ( HException const& e ) {
		ret = e.code();
		log( LOG_LEVEL::ERROR ) << e.what() << endl;
	}
	return ( ret );
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
		info_->fh = _fs_->create( path_, info_->flags, mode_ );
	} catch ( HException const& e ) {
		ret = e.code();
		log( LOG_LEVEL::ERROR ) << e.what() << endl;
	}
	return ( ret );
}

int ftruncate( char const* path_, off_t size_, struct fuse_file_info* info_ ) {
	if ( setup._debug ) {
		log_trace << path_ << ", " << size_ << endl;
	}
	int ret( 0 );
	try {
		_fs_->ftruncate( info_->fh, static_cast<int>( size_ ) );
	} catch ( HException const& e ) {
		ret = e.code();
		log( LOG_LEVEL::ERROR ) << e.what() << endl;
	}
	return ( ret );
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

int write_buf( char const* path_, struct fuse_bufvec* src_, off_t offset_, struct fuse_file_info* info_ ) {
	if ( setup._debug ) {
		int toWrite( static_cast<int>( fuse_buf_size( src_ ) ) );
		log_trace << path_ << ", to write: " << toWrite << endl;
	}
	int ret( 0 );
	try {
		ret = _fs_->write_buf( info_->fh, src_, static_cast<int>( offset_ ) );
		if ( setup._debug ) {
			log_trace << path_ << ", written: " << ret << endl;
		}
	} catch ( HException const& e ) {
		ret = e.code();
		log( LOG_LEVEL::ERROR ) << e.what() << ", " << -e.code() << endl;
	}
	return ( ret );
}

int read_buf( char const* path_, struct fuse_bufvec** dst_, size_t size_, off_t offset_, struct fuse_file_info* info_ ) {
	if ( setup._debug ) {
		log_trace << path_ << ", to read: " << size_ << ", at offset: " << offset_ << endl;
	}
	int ret( 0 );
	try {
		ret = _fs_->read_buf( info_->fh, dst_, static_cast<int>( size_ ), static_cast<int>( offset_ ) );
		if ( setup._debug ) {
			log_trace << path_ << ", read: " << ret << endl;
		}
	} catch ( HException const& e ) {
		ret = e.code();
		log( LOG_LEVEL::ERROR ) << e.what() << ", " << -e.code() << endl;
	}
	return ( ret );
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
	xmlfsFuse.access      = &access;
	xmlfsFuse.bmap        = &bmap;
	xmlfsFuse.chmod       = &chmod;
	xmlfsFuse.chown       = &chown;
	xmlfsFuse.create      = &create;
	xmlfsFuse.destroy     = &destroy;
	xmlfsFuse.fallocate   = &fallocate;
	xmlfsFuse.fgetattr    = &fgetattr;
	xmlfsFuse.flock       = &flock;
	xmlfsFuse.flush       = &flush;
	xmlfsFuse.fsync       = &fsync;
	xmlfsFuse.fsyncdir    = &fsyncdir;
	xmlfsFuse.ftruncate   = &ftruncate;
	xmlfsFuse.getattr     = &getattr;
	xmlfsFuse.getdir      = nullptr;
	xmlfsFuse.getxattr    = &getxattr;
	xmlfsFuse.init        = &init;
	xmlfsFuse.ioctl       = &ioctl;
	xmlfsFuse.link        = &link;
	xmlfsFuse.listxattr   = &listxattr;
	xmlfsFuse.lock        = &lock;
	xmlfsFuse.mkdir       = &mkdir;
	xmlfsFuse.mknod       = &mknod;
	xmlfsFuse.open        = &open;
	xmlfsFuse.opendir     = &opendir;
	xmlfsFuse.poll        = &poll;
	xmlfsFuse.read        = &read;
	xmlfsFuse.read_buf    = &read_buf;
	xmlfsFuse.readdir     = &readdir;
	xmlfsFuse.readlink    = &readlink;
	xmlfsFuse.release     = &release;
	xmlfsFuse.releasedir  = &releasedir;
	xmlfsFuse.removexattr = &removexattr;
	xmlfsFuse.rename      = &rename;
	xmlfsFuse.rmdir       = &rmdir;
	xmlfsFuse.setxattr    = &setxattr;
	xmlfsFuse.statfs      = &statfs;
	xmlfsFuse.symlink     = &symlink;
	xmlfsFuse.truncate    = &truncate;
	xmlfsFuse.unlink      = &unlink;
	xmlfsFuse.utime       = nullptr;
	xmlfsFuse.utimens     = &utimens;
	xmlfsFuse.write       = &write;
	xmlfsFuse.write_buf   = &write_buf;
	return ( fuse_main( argc_, argv_, &xmlfsFuse, nullptr ) );
}

}

