/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdlib.h>
#include <AP_GST/AP_GST.h>
//TODO:
// * support for multiple read and write
// * support for strerr
// * ability to lock memory
// * deplay callback system on write
int8_t AP_GST::open(struct gst_metadata *meta, enum open_type fdt)
{
    struct rdwr_info rdwr;
    switch(fdt) {
        case O_WRITE: {
            //Create node
            write_node *new_node;
            new_node = new(write_node);
            new_node->meta = meta;
            new_node->data = malloc(meta->o_size);

            //Store node
            write_list.push_back(new_node);

            //update fd list
            rdwr.fd = last_write_fd;
            rdwr.fd_type = O_WRITE;
            rdwr_info_list.push_back(rdwr);
            last_write_fd++;
            return last_rdwr_fd++;
        }
        case O_READ: {
            //Find appropriate write node
            read_node *new_node;
            int8_t write_fd = -1;            
            for(uint8_t i=0; i<last_write_fd; i++) {
                if(strcmp(meta->o_name, write_list[i]->meta->o_name) == 0) {
                    write_fd = i;
                    break;
                }
            }

            //Create read node
            if(write_fd != -1) {
                new_node = new(read_node);
                new_node->meta = meta;
                new_node->write_fd = write_fd;
                read_list.push_back(new_node);
            } else {
                return -1;
            }

            //update fd list
            rdwr.fd = last_read_fd;
            rdwr.fd_type = O_READ;
            rdwr_info_list.push_back(rdwr);
            last_read_fd++;
            return last_rdwr_fd++;
        }
        default:
        return -1;
    }
}
// 
int8_t AP_GST::write(int8_t fd, const void* data)
{
    if(fd > last_rdwr_fd || rdwr_info_list[fd].fd_type != O_WRITE) {
        return -1;
    }

    //convert rdwr fd to write fd
    fd = rdwr_info_list[last_rdwr_fd].fd;
    //write data to memory
    memcpy(write_list[fd]->data, data,  write_list[fd]->meta->o_size);
    //raise updated flags of relevant read nodes
    for(uint8_t i = 0; i < last_read_fd; i++) {
        if(strcmp(read_list[fd]->meta->o_name,write_list[fd]->meta->o_name)) {
            read_list[fd]->updated = 0;
        }
    }
    return 0;
}

int8_t AP_GST::read(int8_t fd, void* data)
{
    if(fd > last_rdwr_fd || rdwr_info_list[fd].fd_type != O_READ) {
        return -1;
    }
    //convert rdwr fd to read fd
    fd = rdwr_info_list[last_rdwr_fd].fd;
    int8_t write_fd = read_list[fd]->write_fd;
    //check for fd sanity
    if(strcmp(read_list[fd]->meta->o_name,write_list[write_fd]->meta->o_name) == 0) {
        return -1;
    }
    //write data to memory
    memcpy(data, write_list[write_fd]->data, write_list[write_fd]->meta->o_size);
    read_list[fd]->updated = -1;
    return 0;
}

int8_t AP_GST::available(int8_t fd)
{
    if(fd > last_rdwr_fd || rdwr_info_list[last_rdwr_fd].fd_type != O_READ) {
        return -1;
    }
    //convert rdwr fd to read fd
    fd = rdwr_info_list[last_rdwr_fd].fd;
    return read_list[fd]->updated;
}