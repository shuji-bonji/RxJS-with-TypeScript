---
description: "De every operator evalueert of alle waarden aan een opgegeven voorwaarde voldoen en retourneert false zodra de eerste waarde niet voldoet, wat kortsluitende evaluatie mogelijk maakt. Implementeert validatie, datakwaliteitscontroles en stream-verwerking equivalent aan Array.every() op een type-veilige manier in TypeScript."
---

# every - Alle Voldoen aan Voorwaarde

De `every` operator evalueert of alle waarden die door de bron Observable worden uitgezonden aan een opgegeven voorwaarde voldoen,
en **eindigt met het retourneren van `false` zodra de eerste waarde niet voldoet**. Als alle waarden voldoen, wordt `true` geretourneerd.

## 🔰 Basissyntax en Werking

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
// Output: false (stopt bij 5)
```

[🌐 RxJS Officiële Documentatie - every](https://rxjs.dev/api/index/function/every)

## 💡 Typische Gebruiksvoorbeelden

- **Validatiecontroles**: Verifiëren dat alle voorwaarden zijn vervuld
- **Batch-invoer validatie**: Scenario's waarin meerdere waarden collectief worden geëvalueerd
- **In tegenstelling tot array filters, effectief voor het direct controleren van de algehele tevredenheid**

## 🧪 Praktische Code Voorbeelden (met UI)

### ✅ 1. Bepalen of een Array Alleen Even Getallen Bevat

```ts
import { from } from 'rxjs';
import { every } from 'rxjs';

const container = document.createElement('div');
container.innerHTML = '<h3>every operator voorbeeld:</h3>';
document.body.appendChild(container);

const allEvenButton = document.createElement('button');
allEvenButton.textContent = 'Alleen even [2, 4, 6, 8]';
container.appendChild(allEvenButton);

const someOddButton = document.createElement('button');
someOddButton.textContent = 'Bevat oneven [2, 4, 5, 8]';
container.appendChild(someOddButton);

const result = document.createElement('div');
result.style.marginTop = '10px';
result.style.padding = '10px';
result.style.border = '1px solid #ccc';
container.appendChild(result);

allEvenButton.addEventListener('click', () => {
  result.textContent = 'Beoordelen...';
  from([2, 4, 6, 8])
    .pipe(every((x) => x % 2 === 0))
    .subscribe((res) => {
      result.textContent = `Allemaal even?: ${res}`;
      result.style.color = res ? 'green' : 'red';
    });
});

someOddButton.addEventListener('click', () => {
  result.textContent = 'Beoordelen...';
  from([2, 4, 5, 8])
    .pipe(every((x) => x % 2 === 0))
    .subscribe((res) => {
      result.textContent = `Allemaal even?: ${res}`;
      result.style.color = res ? 'green' : 'red';
    });
});
```

### ✅ 2. Toepassen in Formuliervalidatie

```ts
import { combineLatest, fromEvent } from 'rxjs';
import { map, startWith, every, tap } from 'rxjs';

// UI-elementen aanmaken
const formContainer = document.createElement('div');
formContainer.innerHTML = '<h3>Formuliervalidatie met every:</h3>';
document.body.appendChild(formContainer);

// Formulier aanmaken
const form = document.createElement('form');
form.style.padding = '15px';
form.style.border = '1px solid #ddd';
form.style.borderRadius = '5px';
formContainer.appendChild(form);

// Naam invoer
const nameLabel = document.createElement('label');
nameLabel.textContent = 'Naam: ';
nameLabel.style.display = 'block';
nameLabel.style.marginBottom = '5px';
form.appendChild(nameLabel);

const nameInput = document.createElement('input');
nameInput.type = 'text';
nameInput.style.width = '100%';
nameInput.style.padding = '5px';
nameInput.style.marginBottom = '15px';
form.appendChild(nameInput);

// Leeftijd invoer
const ageLabel = document.createElement('label');
ageLabel.textContent = 'Leeftijd: ';
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

// E-mail invoer
const emailLabel = document.createElement('label');
emailLabel.textContent = 'E-mail: ';
emailLabel.style.display = 'block';
emailLabel.style.marginBottom = '5px';
form.appendChild(emailLabel);

const emailInput = document.createElement('input');
emailInput.type = 'email';
emailInput.style.width = '100%';
emailInput.style.padding = '5px';
emailInput.style.marginBottom = '15px';
form.appendChild(emailInput);

// Verstuur knop
const submitButton = document.createElement('button');
submitButton.type = 'submit';
submitButton.textContent = 'Versturen';
submitButton.disabled = true;
form.appendChild(submitButton);

// Validatiebericht
const validationMessage = document.createElement('div');
validationMessage.style.marginTop = '10px';
validationMessage.style.color = 'red';
formContainer.appendChild(validationMessage);

// Naam validatie
const nameValid$ = fromEvent(nameInput, 'input').pipe(
  map((event) => {
    const value = (event.target as HTMLInputElement).value.trim();
    return value.length >= 2;
  }),
  startWith(false)
);

// Leeftijd validatie
const ageValid$ = fromEvent(ageInput, 'input').pipe(
  map((event) => {
    const value = Number((event.target as HTMLInputElement).value);
    return !isNaN(value) && value > 0 && value < 120;
  }),
  startWith(false)
);

// E-mail validatie
const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
const emailValid$ = fromEvent(emailInput, 'input').pipe(
  map((event) => {
    const value = (event.target as HTMLInputElement).value.trim();
    return emailRegex.test(value);
  }),
  startWith(false)
);

// Validatie van alle velden
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
        'Vul alle velden correct in';
    }
  });

// Formulier verzenden
form.addEventListener('submit', (event) => {
  event.preventDefault();

  const formData = {
    name: nameInput.value,
    age: ageInput.value,
    email: emailInput.value,
  };

  validationMessage.textContent = 'Formulier verzonden!';
  validationMessage.style.color = 'green';

  console.log('Verzonden gegevens:', formData);
});

```
