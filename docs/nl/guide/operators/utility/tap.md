---
description: De tap operator is een utility operator die bijwerkingen laat uitvoeren zonder de waarde van de stream te beïnvloeden. Ideaal voor debuggen met logoutput, het controleren van laadstatussen, analysetracking, foutmonitoring en andere toepassingen waar externe verwerking wordt uitgevoerd terwijl de stream wordt geobserveerd. Bijwerkingen kunnen worden beheerd in declaratieve code met behoud van TypeScript's type-veiligheid.
---

# tap - Voer bijwerkingen uit

De `tap` operator wordt gebruikt om "bijwerkingen uit te voeren zonder de stream te wijzigen."
Ideaal voor logging, debuggen of andere operaties die geen invloed hebben op waarden.

## 🔰 Basissyntax en werking

Gebruikt in situaties waar u alleen bijwerkingen wilt toevoegen zonder de waardestroom te wijzigen.

```ts
import { of, tap } from 'rxjs';

of(42).pipe(
  tap(value => console.log('tap:', value))
).subscribe();
// Uitvoer:
// tap: 42
```

In dit voorbeeld wordt de waarde uitgezonden door `of(42)` gelogd terwijl het door `tap` gaat.
Omdat `tap` de waarde "zoals het is" doorgeeft, heeft het geen effect op de inhoud van de stream.

[🌐 RxJS Officiële Documentatie - tap](https://rxjs.dev/api/index/function/tap)

## 💡 Typische use cases

`tap` wordt vaak gebruikt voor de volgende doeleinden:

- Debuggen en logging
- Schakelen van laadstatus
- Weergeven van toast-meldingen
- Triggeren van UI-updates

```ts
import { of, tap, map } from 'rxjs';

of(Math.random()).pipe(
  tap(val => console.log('Opgehaalde waarde:', val)),
  map(n => n > 0.5 ? 'Hoog' : 'Laag'),
  tap(label => console.log('Label:', label))
).subscribe();
// Uitvoer:
// Opgehaalde waarde: 0.09909888881113504
// Label: Laag
```


## 🧪 Praktisch codevoorbeeld (met UI)

Het volgende is een voorbeeld van het toevoegen van logs aan de DOM met tap.

```ts
import { of } from 'rxjs';
import { tap, map } from 'rxjs';

// Element voor loguitvoer
const logOutput = document.createElement('div');
document.body.appendChild(logOutput);

// Waardenreeks
of(1, 2, 3, 4, 5)
  .pipe(
    tap((val) => {
      console.log(`Originele waarde: ${val}`);

      // Voeg log toe aan UI
      const logEntry = document.createElement('div');
      logEntry.textContent = `tap: Waarde ${val} doorgelaten`;
      logEntry.style.color = '#666';
      logOutput.appendChild(logEntry);
    }),
    map((val) => val * 10),
    tap((val) => {
      console.log(`Getransformeerde waarde: ${val}`);

      // Voeg log toe aan UI
      const logEntry = document.createElement('div');
      logEntry.textContent = `tap: Getransformeerde waarde ${val}`;
      logEntry.style.color = '#090';
      logOutput.appendChild(logEntry);
    })
  )
  .subscribe((val) => {
    // Toon eindresultaat in UI
    const resultItem = document.createElement('div');
    resultItem.textContent = `Resultaat: ${val}`;
    resultItem.style.fontWeight = 'bold';
    logOutput.appendChild(resultItem);
  });

```


## ✅ Samenvatting

- `tap` is een operator gespecialiseerd in **het invoegen van bijwerkingen**
- **Logoutput en UI-updates** kunnen worden gedaan zonder de waardestroom te wijzigen
- Kan worden gecombineerd met `finalize` en `catchError` voor meer praktische controle
