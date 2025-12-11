---
description: De iif operator is een RxJS conditionele vertakkingsoperator die één van twee Observables selecteert afhankelijk van een conditionele expressie, en kan worden gebruikt als een ternaire operator.
titleTemplate: ':title | RxJS'
---

# iif - Selectie van Observable op basis van conditie

De `iif` operator selecteert één van twee Observables op basis van het resultaat van het evalueren van een conditionele expressie.
De JavaScript ternaire operator (`condition ? trueValue : falseValue`).


## Basissyntax en werking

```ts
import { iif, of } from 'rxjs';

function getData(condition: boolean) {
  return iif(() => condition, of('YES'), of('NO'));
}

getData(true).subscribe(console.log);

// Output:
// YES
```

Geeft `'YES'` terug als de conditie `true` is, `'NO'` als de conditie `false` is.

[🌐 RxJS Officiële Documentatie - iif](https://rxjs.dev/api/index/function/iif)

## Typische Toepassingsvoorbeelden

`iif` wordt vaak gebruikt in combinatie met `EMPTY` om een "geen-issue stream" terug te geven als niet aan de voorwaarde wordt voldaan.

```ts
import { iif, of, EMPTY } from 'rxjs';

function conditionalData(value: number) {
  return iif(
    () => value > 0,
    of(`Positive value: ${value}`),
    EMPTY
  );
}

conditionalData(0).subscribe(console.log);
conditionalData(1).subscribe(console.log);

// Output:
// Positive value: 1
```


## Praktische codevoorbeelden (met UI)

Het volgende codevoorbeeld met UI gebruikt `iif` om dynamisch te schakelen wat er wordt gepubliceerd en of een Observable wel of niet wordt gepubliceerd als reactie op gebruikersacties en numerieke invoer.
Het volgende codevoorbeeld met UI gebruikt `iif` om dynamisch te schakelen wat er wordt uitgegeven of niet wordt uitgegeven door Observable volgens gebruikersbewerking of numerieke invoer.

Een dergelijk patroon is geschikt voor de volgende praktische gebruikssituaties.

- ✅ API-verzoeken onderdrukken op basis van invoerwaarden (bijv. niet verzenden als het nummer kleiner is dan 0)
- ✅ Schermweergave en verwerkingsmodus schakelen volgens configuratievlaggen
- ✅ Bevestiging en modale besturing op basis van voorwaarden

```ts
import { iif, of, EMPTY } from 'rxjs';

function conditionalData(value: number) {
  return iif(() => value > 0, of(`Positive value: ${value}`), EMPTY);
}

// Retourneer verschillende Observables op basis van voorwaarden
function getDataBasedOnCondition(condition: boolean) {
  return iif(() => condition, of('Condition is true'), of('Condition is false'));
}

// Maak UI-elementen aan
const iifContainer = document.createElement('div');
iifContainer.innerHTML = '<h3>iif operator voorbeeld:</h3>';
document.body.appendChild(iifContainer);

const trueButton = document.createElement('button');
trueButton.textContent = 'Uitvoeren met True conditie';
trueButton.style.marginRight = '10px';
iifContainer.appendChild(trueButton);

const falseButton = document.createElement('button');
falseButton.textContent = 'Uitvoeren met False conditie';
iifContainer.appendChild(falseButton);

const iifResult = document.createElement('div');
iifResult.style.marginTop = '10px';
iifResult.style.padding = '10px';
iifResult.style.border = '1px solid #ddd';
iifContainer.appendChild(iifResult);

trueButton.addEventListener('click', () => {
  iifResult.textContent = '';
  getDataBasedOnCondition(true).subscribe((result) => {
    iifResult.textContent = result;
    iifResult.style.color = 'green';
  });
});

falseButton.addEventListener('click', () => {
  iifResult.textContent = '';
  getDataBasedOnCondition(false).subscribe((result) => {
    iifResult.textContent = result;
    iifResult.style.color = 'red';
  });
});

// Voorbeeld van combinatie met EMPTY (conditionele vertakking op nummer)
const emptyContainer = document.createElement('div');
emptyContainer.innerHTML = '<h3>Combinatie van iif en EMPTY:</h3>';
document.body.appendChild(emptyContainer);

const valueInput = document.createElement('input');
valueInput.type = 'number';
valueInput.placeholder = 'Voer een nummer in';
valueInput.style.marginRight = '10px';
emptyContainer.appendChild(valueInput);

const checkButton = document.createElement('button');
checkButton.textContent = 'Uitvoeren';
emptyContainer.appendChild(checkButton);

const emptyResult = document.createElement('div');
emptyResult.style.marginTop = '10px';
emptyResult.style.padding = '10px';
emptyResult.style.border = '1px solid #ddd';
emptyContainer.appendChild(emptyResult);

checkButton.addEventListener('click', () => {
  const value = Number(valueInput.value);
  emptyResult.textContent = '';

  conditionalData(value).subscribe({
    next: (result) => {
      emptyResult.textContent = result;
      emptyResult.style.color = 'green';
    },
    complete: () => {
      if (!emptyResult.textContent) {
        emptyResult.textContent =
          'Een waarde van 0 of minder werd ingevoerd, dus er werd niets uitgegeven';
        emptyResult.style.color = 'gray';
      }
    },
  });
});
```
