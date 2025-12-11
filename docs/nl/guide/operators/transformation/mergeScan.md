---
description: mergeScan is een RxJS conversieoperator die asynchrone accumulatieverwerking uitvoert, waarbij scan en mergeMap gedrag worden gecombineerd. Het is het beste geschikt voor situaties waar asynchrone accumulatie vereist is, zoals cumulatieve aggregatie van API-responsen, het uitvoeren van het volgende verzoek gebaseerd op vorige resultaten, en cumulatieve data-acquisitie over meerdere pagina's tijdens paginering. Het aantal gelijktijdige uitvoeringen kan worden gecontroleerd met de concurrent parameter.
titleTemplate: ':title | RxJS'
---

# mergeScan - Accumulatie met asynchrone verwerking

De `mergeScan` operator voert een **asynchrone accumulatie** uit van elke waarde in de stream.
Het werkt als een combinatie van `scan` en `mergeMap`, waarbij de geaccumuleerde waarden worden bewaard, elke waarde naar een nieuwe Observable wordt geconverteerd, en het resultaat wordt gebruikt voor de volgende accumulatieoperatie.

## 🔰 Basissyntax en gebruik

```ts
import { interval, of } from 'rxjs';
import { mergeScan, take } from 'rxjs';

interval(1000).pipe(
  take(5),
  mergeScan((acc, curr) => {
    // Asynchrone verwerking voor elke waarde (hier onmiddellijk geretourneerd)
    return of(acc + curr);
  }, 0)
).subscribe(console.log);

// Output: 0, 1, 3, 6, 10
```

- `acc` is de cumulatieve waarde, `curr` is de huidige waarde.
- De cumulatieve functie moet **een Observable retourneren**.
- Het resultaat van het verwerken van elke waarde wordt geaccumuleerd.

[🌐 RxJS Officiële Documentatie - `mergeScan`](https://rxjs.dev/api/operators/mergeScan)

## 💡 Typische gebruikspatronen

- Accumuleer en aggregeer API-responsen
- Voer volgend API-verzoek uit gebaseerd op vorige resultaten
- Asynchrone cumulatieve verwerking van realtime data
- Cumulatieve acquisitie van data van meerdere pagina's met paginering

## 📊 Verschil met scan

| Operator | Cumulatieve functie retourwaarde | Gebruiksscenario |
|--------------|------------------|--------------|
| `scan` | Retourneer waarde direct | Synchrone accumulatieverwerking |
| `mergeScan` | Retourneer Observable | Asynchrone accumulatieverwerking |

```ts
// scan - Synchrone verwerking
source$.pipe(
  scan((acc, curr) => acc + curr, 0)
)

// mergeScan - Asynchrone verwerking
source$.pipe(
  mergeScan((acc, curr) => of(acc + curr).pipe(delay(100)), 0)
)
```

## 🧠 Praktisch codevoorbeeld (API cumulatieve acquisitie)

Dit is een voorbeeld waar nieuwe data wordt toegevoegd aan het vorige resultaat elke keer dat een knop wordt geklikt.

```ts
import { fromEvent, of } from 'rxjs';
import { mergeScan, delay, take, map } from 'rxjs';

// Maak knop
const button = document.createElement('button');
button.textContent = 'Data ophalen';
document.body.appendChild(button);

// Maak uitvoergebied
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Dummy API (retourneert data met vertraging)
const fetchData = (page: number) => {
  return of(`Data ${page}`).pipe(delay(500));
};

// Cumulatieve acquisitie bij klikgebeurtenis
fromEvent(button, 'click').pipe(
  take(5), // Maximaal 5 keer
  mergeScan((accumulated, _, index) => {
    const page = index + 1;
    console.log(`Ophalen pagina ${page}...`);

    // Voeg nieuwe data toe aan geaccumuleerde data
    return fetchData(page).pipe(
      map(newData => [...accumulated, newData])
    );
  }, [] as string[])
).subscribe((allData) => {
  output.innerHTML = `
    <div>Opgehaalde data:</div>
    <ul>${allData.map(d => `<li>${d}</li>`).join('')}</ul>
  `;
});
```

- Data wordt asynchroon opgehaald met elke klik.
- Nieuwe data wordt toegevoegd aan het vorige resultaat (`accumulated`).
- **Geaccumuleerde resultaten worden realtime bijgewerkt**.

## 🎯 Praktisch voorbeeld: Accumulatie met gelijktijdigheidscontrole

De `mergeScan` heeft een `concurrent` parameter om het aantal gelijktijdige uitvoeringen te controleren.

```ts
import { interval, of } from 'rxjs';
import { mergeScan, take, delay } from 'rxjs';

interface RequestLog {
  total: number;
  logs: string[];
}

interval(200).pipe(
  take(10),
  mergeScan((acc, curr) => {
    const timestamp = new Date().toLocaleTimeString();
    console.log(`Verzoek ${curr} gestart: ${timestamp}`);

    // Elk verzoek duurt 1 seconde
    return of({
      total: acc.total + 1,
      logs: [...acc.logs, `Verzoek ${curr} voltooid: ${timestamp}`]
    }).pipe(delay(1000));
  }, { total: 0, logs: [] } as RequestLog, 2) // Gelijktijdigheid: 2
).subscribe((result) => {
  console.log(`Totaal: ${result.total} items`);
  console.log(result.logs[result.logs.length - 1]);
});
```

- Met `concurrent: 2` worden maximaal twee verzoeken tegelijk uitgevoerd.
- Het derde en volgende verzoeken wachten tot het vorige verzoek is voltooid.

## ⚠️ Opmerkingen

### 1. Foutafhandeling

Als er een fout optreedt binnen de accumulatiefunctie, stopt de hele stream.

```ts
source$.pipe(
  mergeScan((acc, curr) => {
    return apiCall(curr).pipe(
      map(result => acc + result),
      catchError(err => {
        console.error('Fout opgetreden:', err);
        // Ga door met behoud van geaccumuleerde waarde
        return of(acc);
      })
    );
  }, 0)
)
```

### 2. Geheugenbeheer

Wees voorzichtig dat de cumulatieve waarde niet te groot wordt.

```ts
// Slecht voorbeeld: Onbeperkte accumulatie
mergeScan((acc, curr) => of([...acc, curr]), [])

// Goed voorbeeld: Bewaar alleen de laatste N items
mergeScan((acc, curr) => {
  const newAcc = [...acc, curr];
  return of(newAcc.slice(-100)); // Bewaar alleen de laatste 100 items
}, [])
```

### 3. Gebruik scan voor synchrone verwerking

Als u geen asynchrone verwerking nodig heeft, gebruik eenvoudige `scan`.

```ts
// mergeScan is onnodig
source$.pipe(
  mergeScan((acc, curr) => of(acc + curr), 0)
)

// scan is voldoende
source$.pipe(
  scan((acc, curr) => acc + curr, 0)
)
```

## 🔗 Gerelateerde operators

- [`scan`](/nl/guide/operators/transformation/scan) - Synchrone cumulatieve verwerking
- [`reduce`](/nl/guide/operators/transformation/reduce) - Voert eindcumulatieve waarde alleen uit bij voltooiing
- [`mergeMap`](/nl/guide/operators/transformation/mergeMap) - Asynchrone mapping (geen accumulatie)
- [`expand`](/nl/guide/operators/transformation/expand) - Recursief uitbreidingsproces
