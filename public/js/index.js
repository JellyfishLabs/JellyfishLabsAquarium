import { initializeApp } from 'https://www.gstatic.com/firebasejs/9.12.1/firebase-app.js';
import { getFirestore, collection, getDocs } from 'https://www.gstatic.com/firebasejs/9.12.1/firebase-firestore.js';
// TODO: Add SDKs for Firebase products that you want to use
// https://firebase.google.com/docs/web/setup#available-libraries

import dashboard from './dashboard.js';

export { db };

/* app and database initialization */
const app = initializeApp({
	apiKey: "AIzaSyABeXDEran8ZEJhVkZHGTDJRptGz0aGuLY",
	authDomain: "jellyfishlabsaquarium.firebaseapp.com",
	projectId: "jellyfishlabsaquarium",
	storageBucket: "jellyfishlabsaquarium.appspot.com",
	messagingSenderId: "1067187975037",
	appId: "1:1067187975037:web:159a01af5024d68bdf5214"
});
const db = getFirestore(app);

/* page routing */
const pathToRegex = path => new RegExp("^" + path.replace(/\//g, "\\/").replace(/:\w+/g, "(.+)") + "$");

const getParams = match => {
    const values = match.result.slice(1);
    const keys = Array.from(match.route.path.matchAll(/:(\w+)/g)).map(result => result[1]);

    return Object.fromEntries(keys.map((key, i) => {
        return [key, values[i]];
    }));
};

const router = async () => {
	const routes = [
		{ path: '/', 	 	view: dashboard },
		{ path: 'about', 	view: 'about' 	},
		{ path: 'contact', 	view: 'contact' },
		{ path: 'login',	view: 'login'	}
	];

	const potentialMatches = routes.map(route => {
        return {
            route: route,
            result: location.pathname.match(pathToRegex(route.path))
        };
    });

    let match = potentialMatches.find(potentialMatch => potentialMatch.result !== null);

    if (!match) {
        match = {
            route: routes[0],
            result: [location.pathname]
        };
    }

    const view = new match.route.view(getParams(match));
	document.querySelector('#app').innerHTML = await view.getHTML();
	await view.updateElements();
};

const navigateTo = url => {
	history.pushState({}, '', url);
	router();
};

window.addEventListener('popstate', router);

document.addEventListener('DOMContentLoaded', function() {
	document.body.addEventListener('click', event => {
		if (event.target.matches('[data-link]')) {
			event.preventDefault();
			navigateTo(event.target.href);
		}
	})

	router();
});