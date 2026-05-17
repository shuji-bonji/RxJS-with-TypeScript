---
description: mergeScan ist ein Transformationsoperator von RxJS, der asynchrone kumulative Verarbeitung durchführt und eine Kombination aus scan und mergeMap darstellt. Ideal für Situationen, die asynchrone kumulative Verarbeitung erfordern, wie kumulative Aggregation von API-Antworten, Ausführung der nächsten Anfrage basierend auf vorherigen Ergebnissen oder kumulative Datenerfassung mehrerer Seiten bei Paginierung. Steuerung der gleichzeitigen Ausführungsanzahl mit dem concurrent-Parameter möglich.
---

# mergeScan - Asynchrone Kumulation

Der `mergeScan`-Operator führt **asynchrone kumulative Verarbeitung** für jeden Wert des Streams aus.
Funktioniert wie eine Kombination aus `scan` und `mergeMap`, hält den kumulativen Wert, konvertiert jeden Wert in ein neues Observable und verwendet das Ergebnis für die nächste kumulative Verarbeitung.

## 🔰 Grundlegende Syntax und Verwendung

```ts
import { interval, of } from 'rxjs';
import { mergeScan, take,  } from 'rxjs';

interval(1000).pipe(
  take(5),
  mergeScan((acc, curr) => {
    // Asynchrone Verarbeitung für jeden Wert (hier sofort zurückgegeben)
    return of(acc + curr);
  }, 0)
).subscribe(console.log);

// Ausgabe: 0, 1, 3, 6, 10
```

- `acc` ist der kumulative Wert, `curr` ist der aktuelle Wert.
- Die kumulative Funktion muss **ein Observable zurückgeben**.
- Die Verarbeitungsergebnisse jedes Wertes werden kumuliert.

[🌐 RxJS Offizielle Dokumentation - `mergeScan`](https://rxjs.dev/api/operators/mergeScan)

## 💡 Typische Anwendungsmuster

- Kumulative Aggregation von API-Antworten
- Ausführung der nächsten API-Anfrage basierend auf vorherigen Ergebnissen
- Asynchrone kumulative Verarbeitung von Echtzeitdaten
- Kumulative Datenerfassung mehrerer Seiten bei Paginierung

## 📊 Unterschied zu scan

| Operator | Rückgabewert der kumulativen Funktion | Anwendungsfall |
|--------------|------------------|--------------|
| `scan` | Gibt Wert direkt zurück | Synchrone kumulative Verarbeitung |
| `mergeScan` | Gibt Observable zurück | Asynchrone kumulative Verarbeitung |

```ts
// scan - Synchrone Verarbeitung
source$.pipe(
  scan((acc, curr) => acc + curr, 0)
)

// mergeScan - Asynchrone Verarbeitung
source$.pipe(
  mergeScan((acc, curr) => of(acc + curr).pipe(delay(100)), 0)
)
```

## 🧠 Praktisches Codebeispiel (kumulative API-Erfassung)

Beispiel, das bei jedem Button-Klick neue Daten zu den vorherigen Ergebnissen hinzufügt.

```ts
import { fromEvent, of } from 'rxjs';
import { mergeScan, delay, take, map } from 'rxjs';

// Button erstellen
const button = document.createElement('button');
button.textContent = 'Daten abrufen';
document.body.appendChild(button);

// Ausgabebereich erstellen
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Dummy-API (gibt Daten mit Verzögerung zurück)
const fetchData = (page: number) => {
  return of(`Daten${page}`).pipe(delay(500));
};

// Kumulative Erfassung bei Klick-Ereignis
fromEvent(button, 'click').pipe(
  take(5), // Maximal 5 Mal
  mergeScan((accumulated, _, index) => {
    const page = index + 1;
    console.log(`Seite ${page} wird abgerufen...`);

    // Neue Daten zu bisher kumulierten Daten hinzufügen
    return fetchData(page).pipe(
      map(newData => [...accumulated, newData])
    );
  }, [] as string[])
).subscribe((allData) => {
  output.innerHTML = `
    <div>Abgerufene Daten:</div>
    <ul>${allData.map(d => `<li>${d}</li>`).join('')}</ul>
  `;
});
```

- Bei jedem Klick werden Daten asynchron abgerufen.
- Neue Daten werden zu den bisherigen Ergebnissen (`accumulated`) hinzugefügt.
- **Kumulative Ergebnisse werden in Echtzeit aktualisiert**.

## 🎯 Praktisches Beispiel: Kumulative Verarbeitung mit Parallelitätskontrolle

`mergeScan` hat einen `concurrent`-Parameter zur Steuerung der Anzahl gleichzeitiger Ausführungen.

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
    console.log(`Anfrage${curr} gestartet: ${timestamp}`);

    // Jede Anfrage dauert 1 Sekunde
    return of({
      total: acc.total + 1,
      logs: [...acc.logs, `Anfrage${curr} abgeschlossen: ${timestamp}`]
    }).pipe(delay(1000));
  }, { total: 0, logs: [] } as RequestLog, 2) // Gleichzeitige Ausführung 2
).subscribe((result) => {
  console.log(`Gesamt: ${result.total} Stück`);
  console.log(result.logs[result.logs.length - 1]);
});
```

- Mit `concurrent: 2` werden maximal 2 Anfragen gleichzeitig ausgeführt.
- Ab der 3. Anfrage wird gewartet, bis vorherige Anfragen abgeschlossen sind.

## ⚠️ Achtung

### 1. Fehlerbehandlung

Wenn ein Fehler in der kumulativen Funktion auftritt, stoppt der gesamte Stream.

```ts
source$.pipe(
  mergeScan((acc, curr) => {
    return apiCall(curr).pipe(
      map(result => acc + result),
      catchError((err: unknown) => {
        console.error('Fehler aufgetreten:', err);
        // Kumulativen Wert beibehalten und fortfahren
        return of(acc);
      })
    );
  }, 0)
)
```

### 2. Speicherverwaltung

Darauf achten, dass der kumulative Wert nicht zu groß wird

```ts
// Schlechtes Beispiel: Unbegrenzte Kumulation
mergeScan((acc, curr) => of([...acc, curr]), [])

// Gutes Beispiel: Nur neueste N Einträge behalten
mergeScan((acc, curr) => {
  const newAcc = [...acc, curr];
  return of(newAcc.slice(-100)); // Nur neueste 100
}, [])
```

### 3. Bei synchroner Verarbeitung scan verwenden

Wenn keine asynchrone Verarbeitung erforderlich ist, verwenden Sie das einfachere `scan`.

```ts
// mergeScan ist unnötig
source$.pipe(
  mergeScan((acc, curr) => of(acc + curr), 0)
)

// scan ist ausreichend
source$.pipe(
  scan((acc, curr) => acc + curr, 0)
)
```

## 🔗 Verwandte Operatoren

- [`scan`](./scan.md) - Synchrone kumulative Verarbeitung
- [`reduce`](./reduce.md) - Gibt nur finalen kumulativen Wert bei Abschluss aus
- [`mergeMap`](./mergeMap.md) - Asynchrones Mapping (keine Kumulation)
- [`expand`](./expand.md) - Rekursive Expansionsverarbeitung
