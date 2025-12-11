---
description: De defer operator zorgt ervoor dat de factory-functie van de Observable wordt uitgesteld tot het moment van abonnement. Dit is handig wanneer je een andere waarde of proces wilt evalueren elke keer dat je je abonneert, zoals de huidige tijd, willekeurige waarden, dynamische API-verzoeken, of andere processen waarvan de resultaten veranderen op het moment van uitvoering.
titleTemplate: ':title | RxJS'
---

# defer - Observable-creatie met uitgestelde evaluatie

De `defer` operator voert de Observable factory-functie uit op **het moment van abonnement** en retourneert de resulterende Observable. Hierdoor kun je de creatie van een Observable uitstellen totdat er daadwerkelijk op wordt geabonneerd.

## Basissyntaxis en werking

```ts
import { defer, of } from 'rxjs';

const random$ = defer(() => of(Math.random()));

random$.subscribe(console.log);
random$.subscribe(console.log);

// Output:
// 0.8727962287400634
// 0.8499299688934545
```

In dit voorbeeld wordt `Math.random()` geëvalueerd voor elk abonnement, dus elke keer wordt een andere waarde uitgegeven.

[🌐 RxJS Officiële Documentatie - defer](https://rxjs.dev/api/index/function/defer)

## Typische toepassingsvoorbeelden

Dit is handig wanneer je **processen** wilt uitvoeren zoals API's, externe bronnen, huidige tijd, willekeurige getallen, enz., waarvan de resultaten variëren afhankelijk van het moment van uitvoering.

```ts
import { defer } from 'rxjs';
import { ajax } from 'rxjs/ajax';

function fetchUser(userId: number) {
  return defer(() =>
    ajax.getJSON(`https://jsonplaceholder.typicode.com/users/${userId}`)
  );
}

fetchUser(1).subscribe(console.log);

// Output:
// {id: 1, name: 'Leanne Graham', username: 'Bret', email: 'Sincere@april.biz', address: {…}, …}
```

## Praktische codevoorbeelden (met UI)

`defer` is vooral handig voor processen die bijwerkingen hebben of elke keer verschillende resultaten produceren.

In de onderstaande code kun je ervaren wat het betekent om `defer` te gebruiken om "elke keer dat erop wordt geabonneerd een andere Observable te genereren".
Dit is vooral handig in gevallen** waar je het fetch-proces **elke keer** wilt uitvoeren in plaats van het te cachen.

### ✅ 1. Genereer elke keer een willekeurig getal
```ts
import { defer, of } from 'rxjs';

// Observable dat willekeurige getallen genereert
const randomNumber$ = defer(() => {
  const random = Math.floor(Math.random() * 100);
  return of(random);
});

// Maak UI-elementen aan
const randomContainer = document.createElement('div');
randomContainer.innerHTML = '<h3>Willekeurige waardegeneratie met defer:</h3>';
document.body.appendChild(randomContainer);

// Genereer knop
const generateButton = document.createElement('button');
generateButton.textContent = 'Genereer willekeurige waarde';
randomContainer.appendChild(generateButton);

// Geschiedenisweergavegebied
const randomHistory = document.createElement('div');
randomHistory.style.marginTop = '10px';
randomHistory.style.padding = '10px';
randomHistory.style.border = '1px solid #ddd';
randomHistory.style.maxHeight = '200px';
randomHistory.style.overflowY = 'auto';
randomContainer.appendChild(randomHistory);

// Knopgebeurtenis
generateButton.addEventListener('click', () => {
  randomNumber$.subscribe(value => {
    const entry = document.createElement('div');
    entry.textContent = `Gegenereerde waarde: ${value}`;
    entry.style.padding = '5px';
    entry.style.margin = '2px 0';
    entry.style.backgroundColor = '#f5f5f5';
    entry.style.borderRadius = '3px';
    randomHistory.insertBefore(entry, randomHistory.firstChild);
  });
});

// Uitleg tekst
const randomExplanation = document.createElement('p');
randomExplanation.textContent = 'Elke keer dat je op de knop "Genereer willekeurige waarde" klikt, wordt er een nieuwe willekeurige waarde gegenereerd. Als je normale of gebruikt, wordt de waarde slechts één keer aan het begin gegenereerd, maar door defer te gebruiken, kun je elke keer een nieuwe waarde genereren.';
randomContainer.appendChild(randomExplanation);
```

### ✅ 2. Voer elk API-verzoek uit

Omdat `defer` elke keer dat erop wordt geabonneerd een nieuwe Observable creëert, is het vooral handig in situaties waarin je verschillende API-verzoeken wilt uitvoeren op basis van **gebruikersinvoer, enz.**.
Gebruik bijvoorbeeld het volgende scenario.

- ✅ Ophalen op verschillende URL's afhankelijk van dynamische query's of parameters
- ✅ Elke keer de nieuwste gegevens ophalen** zonder de cache te gebruiken
- ✅ Wil processing lui evalueren wanneer een gebeurtenis plaatsvindt

```ts
import { defer } from 'rxjs';
import { ajax } from 'rxjs/ajax';

const container = document.createElement('div');
container.innerHTML = '<h3>API-verzoek met defer:</h3>';
document.body.appendChild(container);

// Invoerveld
const input = document.createElement('input');
input.placeholder = 'Voer gebruikers-ID in';
container.appendChild(input);

// Uitvoerknop
const button = document.createElement('button');
button.textContent = 'Haal gebruikersinformatie op';
container.appendChild(button);

// Resultaatweergave
const resultBox = document.createElement('pre');
resultBox.style.border = '1px solid #ccc';
resultBox.style.padding = '10px';
resultBox.style.marginTop = '10px';
container.appendChild(resultBox);

// Knopgebeurtenis
button.addEventListener('click', () => {
  const userId = input.value.trim();
  if (!userId) {
    resultBox.textContent = 'Voer alstublieft gebruikers-ID in';
    return;
  }

  const user$ = defer(() =>
    ajax.getJSON(`https://jsonplaceholder.typicode.com/users/${userId}`)
  );

  resultBox.textContent = 'Laden...';
  user$.subscribe({
    next: (data) => (resultBox.textContent = JSON.stringify(data, null, 2)),
    error: (err) => (resultBox.textContent = `Fout: ${err.message}`),
  });
});
```

In dit voorbeeld zorgt de `defer` ervoor dat `ajax.getJSON()` wordt aangeroepen wanneer de gebruiker op de knop drukt,
**`of(ajax.getJSON(...))` In tegenstelling tot de `defer`, die vanaf het begin evalueert, heb je volledige controle** over het moment van uitvoering.
