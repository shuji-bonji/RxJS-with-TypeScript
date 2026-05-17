---
description: "mergeScan est un opérateur de transformation RxJS qui effectue un traitement d'accumulation asynchrone, fonctionnant comme une combinaison de scan et mergeMap. Il est idéal pour les situations nécessitant une accumulation avec traitement asynchrone, comme l'agrégation cumulative de réponses API, l'exécution de requêtes suivantes basées sur les résultats précédents, et l'acquisition cumulative de données multi-pages avec pagination. Le paramètre concurrent permet également de contrôler le nombre d'exécutions simultanées."
---

# mergeScan - Accumulation asynchrone

L'opérateur `mergeScan` effectue un traitement d'accumulation **asynchrone** sur chaque valeur du flux.
Il fonctionne comme une combinaison de `scan` et `mergeMap`, conservant les valeurs accumulées, convertissant chaque valeur en un nouvel Observable et utilisant le résultat pour le prochain traitement d'accumulation.

## 🔰 Syntaxe de base et utilisation

```ts
import { interval, of } from 'rxjs';
import { mergeScan, take,  } from 'rxjs';

interval(1000).pipe(
  take(5),
  mergeScan((acc, curr) => {
    // Traitement asynchrone pour chaque valeur (ici retour immédiat)
    return of(acc + curr);
  }, 0)
).subscribe(console.log);

// Sortie : 0, 1, 3, 6, 10
```

- `acc` est la valeur accumulée, `curr` est la valeur courante.
- La fonction d'accumulation doit **retourner un Observable**.
- Le résultat du traitement de chaque valeur est accumulé.

[🌐 Documentation officielle RxJS - `mergeScan`](https://rxjs.dev/api/operators/mergeScan)

## 💡 Patterns d'utilisation typiques

- Accumuler et agréger les réponses API
- Exécuter la requête API suivante basée sur les résultats précédents
- Traitement d'accumulation asynchrone de données en temps réel
- Acquisition cumulative de données provenant de plusieurs pages avec pagination

## 📊 Différence avec scan

| Opérateur | Valeur de retour de la fonction d'accumulation | Cas d'utilisation |
|--------------|------------------|--------------|
| `scan` | Retourne directement une valeur | Traitement d'accumulation synchrone |
| `mergeScan` | Retourne un Observable | Traitement d'accumulation asynchrone |

```ts
// scan - traitement synchrone
source$.pipe(
  scan((acc, curr) => acc + curr, 0)
)

// mergeScan - traitement asynchrone
source$.pipe(
  mergeScan((acc, curr) => of(acc + curr).pipe(delay(100)), 0)
)
```

## 🧠 Exemple de code pratique (acquisition cumulative API)

Voici un exemple dans lequel de nouvelles données sont ajoutées au résultat précédent chaque fois qu'un bouton est cliqué.

```ts
import { fromEvent, of } from 'rxjs';
import { mergeScan, delay, take, map } from 'rxjs';

// Création du bouton
const button = document.createElement('button');
button.textContent = 'Récupérer les données';
document.body.appendChild(button);

// Création de la zone de sortie
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// API factice (retourne des données avec délai)
const fetchData = (page: number) => {
  return of(`Données${page}`).pipe(delay(500));
};

// Acquisition cumulative au clic
fromEvent(button, 'click').pipe(
  take(5), // Maximum 5 fois
  mergeScan((accumulated, _, index) => {
    const page = index + 1;
    console.log(`Récupération de la page ${page}...`);

    // Ajouter de nouvelles données aux données accumulées précédentes
    return fetchData(page).pipe(
      map(newData => [...accumulated, newData])
    );
  }, [] as string[])
).subscribe((allData) => {
  output.innerHTML = `
    <div>Données récupérées :</div>
    <ul>${allData.map(d => `<li>${d}</li>`).join('')}</ul>
  `;
});
```

- Les données sont récupérées de manière asynchrone à chaque clic.
- De nouvelles données sont ajoutées au résultat précédent (`accumulated`).
- **Les résultats cumulés sont mis à jour en temps réel**.

## 🎯 Exemple pratique : traitement cumulatif avec contrôle de concurrence

`mergeScan` a un paramètre `concurrent` pour contrôler le nombre d'exécutions simultanées.

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
    console.log(`Début de la requête ${curr} : ${timestamp}`);

    // Chaque requête prend 1 seconde
    return of({
      total: acc.total + 1,
      logs: [...acc.logs, `Requête ${curr} terminée : ${timestamp}`]
    }).pipe(delay(1000));
  }, { total: 0, logs: [] } as RequestLog, 2) // Concurrence : 2
).subscribe((result) => {
  console.log(`Cumul : ${result.total} requêtes`);
  console.log(result.logs[result.logs.length - 1]);
});
```

- Avec `concurrent: 2`, maximum 2 requêtes sont exécutées simultanément.
- La 3ème requête et les suivantes attendent que les requêtes précédentes soient terminées.

## ⚠️ Points d'attention

### 1. Gestion des erreurs

Si une erreur se produit dans la fonction d'accumulation, le flux entier s'arrête.

```ts
source$.pipe(
  mergeScan((acc, curr) => {
    return apiCall(curr).pipe(
      map(result => acc + result),
      catchError((err: unknown) => {
        console.error('Erreur survenue :', err);
        // Maintenir la valeur accumulée et continuer
        return of(acc);
      })
    );
  }, 0)
)
```

### 2. Gestion de la mémoire

Veillez à ce que la valeur accumulée ne devienne pas trop importante.

```ts
// Mauvais exemple : accumulation sans limite
mergeScan((acc, curr) => of([...acc, curr]), [])

// Bon exemple : conserver seulement les N derniers éléments
mergeScan((acc, curr) => {
  const newAcc = [...acc, curr];
  return of(newAcc.slice(-100)); // Seulement les 100 derniers
}, [])
```

### 3. Utiliser scan pour le traitement synchrone

Si le traitement asynchrone n'est pas nécessaire, utilisez le simple `scan`.

```ts
// mergeScan n'est pas nécessaire
source$.pipe(
  mergeScan((acc, curr) => of(acc + curr), 0)
)

// scan suffit
source$.pipe(
  scan((acc, curr) => acc + curr, 0)
)
```

## 🔗 Opérateurs associés

- [`scan`](./scan) - Traitement d'accumulation synchrone
- [`reduce`](./reduce) - Émet uniquement la valeur finale accumulée à la fin
- [`mergeMap`](./mergeMap) - Mapping asynchrone (sans accumulation)
- [`expand`](./expand) - Traitement d'expansion récursive
