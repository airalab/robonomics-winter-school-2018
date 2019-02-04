import Vue from 'vue'
import Router from 'vue-router'
import All from '@/components/All'
import Model from '@/components/Model'
import Market from '@/components/market/Market'

Vue.use(Router)

export default new Router({
  routes: [
    {
      path: '/',
      name: 'All',
      component: All
    },
    {
      path: '/model',
      name: 'Model',
      component: Model
    },
    {
      path: '/market',
      name: 'Market',
      component: Market
    }
  ]
})
