---
description: switchMap is een conversieoperator die de vorige Observable annuleert en overschakelt naar de nieuwste. Het is ideaal voor use cases zoals live zoeken, navigatie wisselen, automatisch opslaan, etc. Samen met TypeScript type-inferentie maakt het veilige asynchrone verwerking mogelijk. Het biedt ook gedetailleerde uitleg over hoe het te gebruiken in combinatie met mergeMap en concatMap.
titleTemplate: ':title'
---

# switchMap - Annuleer vorige Observable en schakel naar de nieuwste

De `switchMap` operator maakt een nieuwe Observable voor elke waarde in de invoerstroom, **annuleert de vorige Observable en schakelt alleen naar de meest recente**.
Dit is ideaal voor gevallen waar alleen de meest recente invoer geldig moet zijn, zoals in een zoekformulier.

## 🔰 Basissyntax en gebruik

```ts
import { of } from 'rxjs';
import { delay, switchMap } from 'rxjs';

of('A', 'B', 'C').pipe(
  switchMap(value =>
    of(`${value} voltooid`).pipe(delay(1000))
  )
).subscribe(console.log);

// Output voorbeeld:
// C voltooid
```

- Maak een nieuwe Observable voor elke waarde.
- Echter, **het moment dat een nieuwe waarde binnenkomt, wordt de vorige Observable geannuleerd**.
- Alleen `C` zal uiteindelijk worden uitgevoerd.

[🌐 RxJS Officiële Documentatie - `switchMap`](https://rxjs.dev/api/operators/switchMap)

## 💡 Typische gebruikspatronen

- Automatische aanvulling van invoerformulieren
- Live zoekfunctie (alleen de laatste invoer is geldig)
- Resource laden bij navigatie of routewisseling
- Gebruikersacties overschakelen naar de nieuwste

## 🧠 Praktisch codevoorbeeld (met UI)

Wanneer een gebruiker tekst invoert in het zoekveld, wordt onmiddellijk een API-verzoek verzonden, waarbij **resultaten van alleen de laatste invoer** worden weergegeven.

```ts
import { fromEvent } from 'rxjs';
import { debounceTime, map, switchMap } from 'rxjs';
import { ajax } from 'rxjs/ajax';
import { of } from 'rxjs';

// Maak invoerveld
const searchInput = document.createElement('input');
searchInput.placeholder = 'Zoek op gebruikersnaam';
document.body.appendChild(searchInput);

// Uitvoergebied
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Invoergebeurtenis verwerking
fromEvent(searchInput, 'input').pipe(
  debounceTime(300),
  map(event => (event.target as HTMLInputElement).value.trim()),
  switchMap(term => {
    if (term === '') {
      return of([]);
    }
    return ajax.getJSON(`https://jsonplaceholder.typicode.com/users?username_like=${term}`);
  })
).subscribe(users => {
  output.innerHTML = '';

  (users as any[]).forEach(user => {
    const div = document.createElement('div');
    div.textContent = user.username;
    output.appendChild(div);
  });
});
```

- Elke keer dat de invoer verandert, wordt het vorige verzoek geannuleerd.
- Alleen gebruikers die overeenkomen met de meest recente zoekterm worden weergegeven.
