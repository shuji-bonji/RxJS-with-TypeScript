---
description: L'operatore every valuta se tutti i valori soddisfano una condizione specificata e può eseguire una valutazione short-circuit restituendo false non appena viene incontrato il primo valore che non soddisfa la condizione.
---

# every - Controlla se Tutti i Valori Soddisfano la Condizione

L'operatore `every` valuta se tutti i valori emessi dall'Observable sorgente soddisfano una condizione specificata, e **restituisce `false` e termina non appena viene incontrato il primo valore che non soddisfa la condizione**. Se tutti i valori soddisfano la condizione, viene restituito `true`.

## 🔰 Sintassi e Operazione Base

```ts
import { from } from 'rxjs';
import { every } from 'rxjs';

from([2, 4, 6, 8])
  .pipe(
    every((x) => x % 2 === 0)
  )
  .subscribe(console.log);
// Output: true
```

```ts
import { from } from 'rxjs';
import { every } from 'rxjs';

from([2, 4, 5, 8])
  .pipe(
    every((x) => x % 2 === 0)
  )
  .subscribe(console.log);
// Output: false (si ferma a 5)
```

[🌐 Documentazione Ufficiale RxJS - every](https://rxjs.dev/api/index/function/every)

## 💡 Esempi di Utilizzo Tipici

- **Controllo validazione**: Confermare che tutte le condizioni siano soddisfatte
- **Validazione input massivo**: Valutare più valori insieme
- **Diverso dal filter di array, utile per controllare la soddisfazione complessiva in una volta**

## 🧪 Esempi di Codice Pratici (con UI)

### ✅ 1. Determinare se l'Array è Tutto Numeri Pari

```ts
import { from } from 'rxjs';
import { every } from 'rxjs';

const container = document.createElement('div');
container.innerHTML = '<h3>Esempio operatore every:</h3>';
document.body.appendChild(container);

const allEvenButton = document.createElement('button');
allEvenButton.textContent = 'Solo pari [2, 4, 6, 8]';
container.appendChild(allEvenButton);

const someOddButton = document.createElement('button');
someOddButton.textContent = 'Contiene dispari [2, 4, 5, 8]';
container.appendChild(someOddButton);

const result = document.createElement('div');
result.style.marginTop = '10px';
result.style.padding = '10px';
result.style.border = '1px solid #ccc';
container.appendChild(result);

allEvenButton.addEventListener('click', () => {
  result.textContent = 'Controllo...';
  from([2, 4, 6, 8])
    .pipe(every((x) => x % 2 === 0))
    .subscribe((res) => {
      result.textContent = `Tutti pari?: ${res}`;
      result.style.color = res ? 'green' : 'red';
    });
});

someOddButton.addEventListener('click', () => {
  result.textContent = 'Controllo...';
  from([2, 4, 5, 8])
    .pipe(every((x) => x % 2 === 0))
    .subscribe((res) => {
      result.textContent = `Tutti pari?: ${res}`;
      result.style.color = res ? 'green' : 'red';
    });
});
```

### ✅ 2. Utilizzo nella Validazione Form

```ts
import { combineLatest, fromEvent } from 'rxjs';
import { map, startWith, every, tap } from 'rxjs';

// Crea elementi UI
const formContainer = document.createElement('div');
formContainer.innerHTML = '<h3>Validazione form con every:</h3>';
document.body.appendChild(formContainer);

// Crea form
const form = document.createElement('form');
form.style.padding = '15px';
form.style.border = '1px solid #ddd';
form.style.borderRadius = '5px';
formContainer.appendChild(form);

// Input nome
const nameLabel = document.createElement('label');
nameLabel.textContent = 'Nome: ';
nameLabel.style.display = 'block';
nameLabel.style.marginBottom = '5px';
form.appendChild(nameLabel);

const nameInput = document.createElement('input');
nameInput.type = 'text';
nameInput.style.width = '100%';
nameInput.style.padding = '5px';
nameInput.style.marginBottom = '15px';
form.appendChild(nameInput);

// Input età
const ageLabel = document.createElement('label');
ageLabel.textContent = 'Età: ';
ageLabel.style.display = 'block';
ageLabel.style.marginBottom = '5px';
form.appendChild(ageLabel);

const ageInput = document.createElement('input');
ageInput.type = 'number';
ageInput.min = '0';
ageInput.style.width = '100%';
ageInput.style.padding = '5px';
ageInput.style.marginBottom = '15px';
form.appendChild(ageInput);

// Input email
const emailLabel = document.createElement('label');
emailLabel.textContent = 'Email: ';
emailLabel.style.display = 'block';
emailLabel.style.marginBottom = '5px';
form.appendChild(emailLabel);

const emailInput = document.createElement('input');
emailInput.type = 'email';
emailInput.style.width = '100%';
emailInput.style.padding = '5px';
emailInput.style.marginBottom = '15px';
form.appendChild(emailInput);

// Bottone invio
const submitButton = document.createElement('button');
submitButton.type = 'submit';
submitButton.textContent = 'Invia';
submitButton.disabled = true;
form.appendChild(submitButton);

// Messaggio validazione
const validationMessage = document.createElement('div');
validationMessage.style.marginTop = '10px';
validationMessage.style.color = 'red';
formContainer.appendChild(validationMessage);

// Validazione nome
const nameValid$ = fromEvent(nameInput, 'input').pipe(
  map((event) => {
    const value = (event.target as HTMLInputElement).value.trim();
    return value.length >= 2;
  }),
  startWith(false)
);

// Validazione età
const ageValid$ = fromEvent(ageInput, 'input').pipe(
  map((event) => {
    const value = Number((event.target as HTMLInputElement).value);
    return !isNaN(value) && value > 0 && value < 120;
  }),
  startWith(false)
);

// Validazione email
const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
const emailValid$ = fromEvent(emailInput, 'input').pipe(
  map((event) => {
    const value = (event.target as HTMLInputElement).value.trim();
    return emailRegex.test(value);
  }),
  startWith(false)
);

// Valida tutti i campi
combineLatest([nameValid$, ageValid$, emailValid$])
  .pipe(
    // tap((v) => console.log(v)),
    map((validList) => validList.every((v) => v === true))
  )
  .subscribe((allValid) => {
    submitButton.disabled = !allValid;
    if (allValid) {
      validationMessage.textContent = '';
    } else {
      validationMessage.textContent =
        'Per favore compila correttamente tutti i campi';
    }
  });

// Invio form
form.addEventListener('submit', (event) => {
  event.preventDefault();

  const formData = {
    name: nameInput.value,
    age: ageInput.value,
    email: emailInput.value,
  };

  validationMessage.textContent = 'Form inviato!';
  validationMessage.style.color = 'green';

  console.log('Dati inviati:', formData);
});
```
