import { collection, setDoc, getDoc, doc } from 'https://www.gstatic.com/firebasejs/9.12.1/firebase-firestore.js';
import { db } from '../js/index.js'
import common from './common.js'    

export default class extends common {
    constructor() {
        super();
    }

    async getHTML() {
        return await fetch('../html/dashboard.html').then((response) => response.text());
    }

    async updateElements() {
        document.getElementById('temp-update').addEventListener('click', (event) => { updateControl('Target', 'Temperature', document.getElementById('temp-input').value); });
        document.getElementById('pH-update').addEventListener('click', (event) => { updateControl('Target', 'pH', event.target.value); });
        //document.getElementById('LED-update').addEventListener('click', () => { UpdateLED() });

        const docRef = doc(db, 'Controls', 'Target');
        const docSnap = await getDoc(docRef);

        if (docSnap.exists()) {
            console.log(docSnap.data());
        }
    }
}

/* periodic tasks */
// var interval = setInterval(function() { updateCurrentValues(); }, 5000);

// async function updateCurrentValues() {
//     let json = await readDatabase('Controls', 'Current');
//     try {
//         document.getElementById('temp-current').textContent = json["Temperature"];
//         document.getElementById('pH-current').textContent = json["pH"];
//     } catch (e) {
//         console.error('Error: Unable to read field from database');
//     }
// }

async function readDatabase(target_col, target_doc) {
    const docRef = doc(db, target_col, target_doc);
    const docSnap = await getDoc(docRef);

    if (docSnap.exists()) {
        console.log('Following data successfully read:');
        console.log(docSnap.data());
        return docSnap.data();
    }
    else {
        console.error('Error: Failed to read from database');
        return null;
    }
}

async function writeDatabase(target_col, target_doc, data) {
    await setDoc(doc(db, target_col, target_doc), data);
    console.log('Following data successfully written:');
    console.log(data);
}

async function updateControl(target_doc, field, new_val) {
    let json = await readDatabase('Controls', target_doc);
    try {
        json[field] = new_val;
    } catch (e) {
        console.error('Error: Attempt to write to non-existent field');
    }
    await writeDatabase('Controls', target_doc, json);
    
    document.getElementById('temp-target').textContent = new_val;
}