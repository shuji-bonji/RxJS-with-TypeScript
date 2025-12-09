---
description: "Der every-Operator bewertet, ob alle Werte eine bestimmte Bedingung erfüllen, und ermöglicht eine kurzschließende Bewertung, die false zurückgibt, sobald ein Wert die Bedingung nicht erfüllt. Implementieren Sie Validierung, Datenqualitätsprüfungen und Stream-Verarbeitung entsprechend Array.every() typsicher mit TypeScript."
---

# every - Überprüfen, ob alle Werte eine Bedingung erfüllen

Der `every`-Operator bewertet, ob alle vom Quell-Observable emittierten Werte eine bestimmte Bedingung erfüllen, und **gibt `false` zurück und beendet sich, sobald ein Wert die Bedingung nicht erfüllt**. Wenn alle Werte die Bedingung erfüllen, wird `true` zurückgegeben.

## 🔰 Grundlegende Syntax und Verhalten

```ts
import { from } from 'rxjs';
import { every } from 'rxjs';

from([2, 4, 6, 8])
  .pipe(
    every((x) => x % 2 === 0)
  )
  .subscribe(console.log);
// Ausgabe: true
```

```ts
import { from } from 'rxjs';
import { every } from 'rxjs';

from([2, 4, 5, 8])
  .pipe(
    every((x) => x % 2 === 0)
  )
  .subscribe(console.log);
// Ausgabe: false (stoppt bei 5)
```

[🌐 RxJS Offizielle Dokumentation - every](https://rxjs.dev/api/index/function/every)

## 💡 Typische Anwendungsfälle

- **Validierungsprüfung**: Überprüfen, ob alle Bedingungen erfüllt sind
- **Stapelvalidierung**: Szenarien, in denen mehrere Werte zusammen bewertet werden
- **Im Gegensatz zu Array-Filtern ist es effektiv, die Gesamtzufriedenheit auf einen Schlag zu überprüfen**

## 🧪 Praktische Codebeispiele (mit UI)

### ✅ 1. Überprüfen, ob ein Array nur gerade Zahlen enthält

```ts
import { from } from 'rxjs';
import { every } from 'rxjs';

const container = document.createElement('div');
container.innerHTML = '<h3>Beispiel für every-Operator:</h3>';
document.body.appendChild(container);

const allEvenButton = document.createElement('button');
allEvenButton.textContent = 'Nur gerade Zahlen [2, 4, 6, 8]';
container.appendChild(allEvenButton);

const someOddButton = document.createElement('button');
someOddButton.textContent = 'Mit ungeraden Zahlen [2, 4, 5, 8]';
container.appendChild(someOddButton);

const result = document.createElement('div');
result.style.marginTop = '10px';
result.style.padding = '10px';
result.style.border = '1px solid #ccc';
container.appendChild(result);

allEvenButton.addEventListener('click', () => {
  result.textContent = 'Bewertung läuft...';
  from([2, 4, 6, 8])
    .pipe(every((x) => x % 2 === 0))
    .subscribe((res) => {
      result.textContent = `Alle gerade?: ${res}`;
      result.style.color = res ? 'green' : 'red';
    });
});

someOddButton.addEventListener('click', () => {
  result.textContent = 'Bewertung läuft...';
  from([2, 4, 5, 8])
    .pipe(every((x) => x % 2 === 0))
    .subscribe((res) => {
      result.textContent = `Alle gerade?: ${res}`;
      result.style.color = res ? 'green' : 'red';
    });
});
```

### ✅ 2. Verwendung bei der Formularvalidierung

```ts
import { combineLatest, fromEvent } from 'rxjs';
import { map, startWith, every, tap } from 'rxjs';

// UI-Elemente erstellen
const formContainer = document.createElement('div');
formContainer.innerHTML = '<h3>Formularvalidierung mit every:</h3>';
document.body.appendChild(formContainer);

// Formular erstellen
const form = document.createElement('form');
form.style.padding = '15px';
form.style.border = '1px solid #ddd';
form.style.borderRadius = '5px';
formContainer.appendChild(form);

// Namenseingabe
const nameLabel = document.createElement('label');
nameLabel.textContent = 'Name: ';
nameLabel.style.display = 'block';
nameLabel.style.marginBottom = '5px';
form.appendChild(nameLabel);

const nameInput = document.createElement('input');
nameInput.type = 'text';
nameInput.style.width = '100%';
nameInput.style.padding = '5px';
nameInput.style.marginBottom = '15px';
form.appendChild(nameInput);

// Alterseingabe
const ageLabel = document.createElement('label');
ageLabel.textContent = 'Alter: ';
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

// E-Mail-Eingabe
const emailLabel = document.createElement('label');
emailLabel.textContent = 'E-Mail: ';
emailLabel.style.display = 'block';
emailLabel.style.marginBottom = '5px';
form.appendChild(emailLabel);

const emailInput = document.createElement('input');
emailInput.type = 'email';
emailInput.style.width = '100%';
emailInput.style.padding = '5px';
emailInput.style.marginBottom = '15px';
form.appendChild(emailInput);

// Absenden-Schaltfläche
const submitButton = document.createElement('button');
submitButton.type = 'submit';
submitButton.textContent = 'Absenden';
submitButton.disabled = true;
form.appendChild(submitButton);

// Validierungsnachricht
const validationMessage = document.createElement('div');
validationMessage.style.marginTop = '10px';
validationMessage.style.color = 'red';
formContainer.appendChild(validationMessage);

// Namensvalidierung
const nameValid$ = fromEvent(nameInput, 'input').pipe(
  map((event) => {
    const value = (event.target as HTMLInputElement).value.trim();
    return value.length >= 2;
  }),
  startWith(false)
);

// Altersvalidierung
const ageValid$ = fromEvent(ageInput, 'input').pipe(
  map((event) => {
    const value = Number((event.target as HTMLInputElement).value);
    return !isNaN(value) && value > 0 && value < 120;
  }),
  startWith(false)
);

// E-Mail-Validierung
const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
const emailValid$ = fromEvent(emailInput, 'input').pipe(
  map((event) => {
    const value = (event.target as HTMLInputElement).value.trim();
    return emailRegex.test(value);
  }),
  startWith(false)
);

// Validierung aller Felder
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
        'Bitte füllen Sie alle Felder korrekt aus';
    }
  });

// Formularübermittlung
form.addEventListener('submit', (event) => {
  event.preventDefault();

  const formData = {
    name: nameInput.value,
    age: ageInput.value,
    email: emailInput.value,
  };

  validationMessage.textContent = 'Formular wurde abgesendet!';
  validationMessage.style.color = 'green';

  console.log('Übermittelte Daten:', formData);
});

```
