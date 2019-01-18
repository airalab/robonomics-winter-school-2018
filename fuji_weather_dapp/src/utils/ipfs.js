// import Ipfs from 'ipfs'
import Promise from 'bluebird'
import axios from 'axios'
import _has from 'lodash/has'
import { IPFS_CONFIG } from '../config'

let ipfs = null
const getIpfs = () => {
  if (ipfs !== null) {
    return Promise.resolve(ipfs)
  }
  const init = (resolve, reject) => {
    ipfs = new Ipfs(IPFS_CONFIG)
    ipfs.once('ready', () => ipfs.id((err, info) => {
      if (err) {
        return reject(err)
      }
      console.log('ipfs id ' + info.id)
      window.ipfsi = ipfs
      resolve(ipfs)
    }))
  }
  const check = (resolve, reject) => {
    if (typeof window !== 'undefined' && window.ipfs) {
      window.ipfs.swarm.peers().then(() => {
        ipfs = window.ipfs
        resolve(ipfs)
      })
        .catch(() => {
          console.log('not glogal ipfs')
          init(resolve, reject)
        })
    } else {
      init(resolve, reject)
    }
  }
  if (document.readyState !== 'complete') {
    return new Promise((resolve, reject) => {
      window.addEventListener('load', () => {
        check(resolve, reject)
      })
    })
  } else {
    return new Promise((resolve, reject) => {
      check(resolve, reject)
    })
  }
}

export default getIpfs

function pendingPromise () { return new Promise(() => { }) }
function killRejected (promise) { return promise.catch(pendingPromise) }
function raceToSuccess (promises) {
  return Promise.race(promises.map(killRejected))
}
export const ipfsGateway = (hash) => (
  axios.get(`https://ipfs.io/ipfs/${hash}`, {
    responseType: 'blob'
  })
    .then(r => r.data)
)
export const cat = (hash) => {
  const list = [ipfsGateway(hash)]
  if (_has(ipfs, 'cat')) {
    list.push(ipfs.cat(hash))
  } else if (_has(ipfs, 'files') && _has(ipfs.files, 'cat')) {
    list.push(ipfs.files.cat(hash))
  }
  return raceToSuccess(list)
}
