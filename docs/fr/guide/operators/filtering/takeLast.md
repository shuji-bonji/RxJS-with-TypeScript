---
description: "takeLast est un opérateur de filtrage RxJS qui ne produit que les N dernières valeurs lorsqu'un flux Observable est terminé. Il est idéal pour les situations où seule la dernière valeur de l'ensemble du flux est requise, comme l'obtention du dernier décompte dans le journal, l'affichage des N premières valeurs dans le tableau de classement, ou le résumé final des données à la fin. Cette fonction ne peut pas être utilisée avec des flux infinis, car elle est conservée dans un tampon jusqu'à ce qu'elle soit terminée."
---

# takeLast - obtient les N dernières valeurs

L'opérateur `takeLast` ne sort que les N dernières valeurs au moment où le flux est **complet**. Il conserve les valeurs dans un tampon jusqu'à ce que le flux se termine et les restitue ensemble une fois terminé.

## 🔰 Syntaxe de base et utilisation

```ts
import { range } from 'rxjs';
import { takeLast } from 'rxjs';

const numbers$ = range(0, 10); // 0de (à)9à

numbers$.pipe(
  takeLast(3)
).subscribe(console.log);
// Sortie: 7, 8, 9
```

**Flux des opérations** :.
1. le flux émet 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
2. maintien interne des 3 derniers dans la mémoire tampon
3. flux terminé 4. valeurs de la mémoire tampon 7, 8, 9
4. sortie des valeurs 7, 8, 9 de la mémoire tampon dans l'ordre

[🌐 Official RxJS documentation - `takeLast`](https://rxjs.dev/api/operators/takeLast)

## 🆚 Contraste avec take.

`take` et `takeLast` ont des comportements opposés.

```ts
import { range } from 'rxjs';
import { take, takeLast } from 'rxjs';

const numbers$ = range(0, 10); // 0de (à)9à

// take: Le premierNObtenir le premier
numbers$.pipe(
  take(3)
).subscribe(console.log);
// Sortie: 0, 1, 2(sortie immédiate)

// takeLast: Obtenir le dernierNObtenir le premier
numbers$.pipe(
  takeLast(3)
).subscribe(console.log);
// Sortie: 7, 8, 9(attendre l'achèvement avant d'éditer)
```

```ts
import { range } from 'rxjs';
import { takeLast } from 'rxjs';

const numbers$ = range(0, 10); // 0de (à)9à

numbers$.pipe(
  takeLast(3)
).subscribe(console.log);
// Sortie: 7, 8, 9

## 💡 Modèle d'utilisation typique

1. **Obtenir les N dernières entrées du journal**.

```ts
   import { from } from 'rxjs';
   import { takeLast } from 'rxjs';

   interface LogEntry {
     timestamp: number;
     level: 'info' | 'warn' | 'error';
     message: string;
   }

   const logs$ = from([
     { timestamp: 1, level: 'info' as const, message: 'App started' },
     { timestamp: 2, level: 'info' as const, message: 'User logged in' },
     { timestamp: 3, level: 'warn' as const, message: 'Slow query detected' },
     { timestamp: 4, level: 'error' as const, message: 'Connection failed' },
     { timestamp: 5, level: 'info' as const, message: 'Retry successful' },
   ] as LogEntry[]);

   // Obtenir le dernier3Récupérer les journaux du
   logs$.pipe(
     takeLast(3)
   ).subscribe(log => {
     console.log(`[${log.level}] ${log.message}`);
   });
   // Sortie:
   // [warn] Slow query detected
   // [error] Connection failed
   // [info] Retry successful
   ```

2. **Haut du classementNRécupérer le haut du classement**
   ```ts
   import { from } from 'rxjs';
   import { takeLast } from 'rxjs';

   interface Score {
     player: string;
     score: number;
   }

   const scores$ = from([
     { player: 'Alice', score: 100 },
     { player: 'Bob', score: 150 },
     { player: 'Charlie', score: 200 },
     { player: 'Dave', score: 180 },
     { player: 'Eve', score: 220 }
   ] as Score[]).pipe(
     // En supposant qu'il soit trié par score
   );

   // Obtenir le top3Récupérer le top du classement
   scores$.pipe(
     takeLast(3)
   ).subscribe(score => {
     console.log(`${score.player}: ${score.score}`);
   });
   // Sortie: Charlie: 200, Dave: 180, Eve: 220
   ```

3. **Résumé final après traitement des donnéesNRésumé des cas**
   ```ts
   import { interval } from 'rxjs';
   import { take, map, takeLast } from 'rxjs';

   // Simulation des données des capteurs
   const sensorData$ = interval(100).pipe(
     take(20),
     map(i => ({
       id: i,
       temperature: 20 + Math.random() * 10
     }))
   );

   // Obtenir le dernier5Calcul de la température moyenne du boîtier
   sensorData$.pipe(
     takeLast(5)
   ).subscribe({
     next: data => {
       console.log(`données${data.id}: ${data.temperature.toFixed(1)}°C`);
     },
     complete: () => {
       console.log('Dernière acquisition5Acquisition des données du cas terminée');
     }
   });
   ```

## 🧠 Exemple de code pratique (historique des entrées)

Exemple d'affichage des dernières valeurs3Il s'agit d'un exemple d'affichage des dernières valeurs saisies par l'utilisateur.

```

ts.
import { fromEvent, Subject } from 'rxjs' ;
import { takeLast } from 'rxjs' ;

// Création d'éléments d'interface utilisateur
const container = document.createElement('div') ;
document.body.appendChild(container) ;

const input = document.createElement('input') ;
input.placeholder = 'Saisir une valeur et Enter' ;
container.appendChild(input) ;

const submitButton = document.createElement('button') ;
submitButton.textContent = 'Afficher l'historique (3 derniers)' ;
container.appendChild(submitButton) ;

const historyDisplay = document.createElement('div') ;
historyDisplay.style.marginTop = '10px' ;
container.appendChild(historyDisplay) ;

// Subject pour contenir les valeurs d'entrée
const inputs$ = new Subject() ;.

// **IMPORTANT** : définir l'abonnement takeLast en premier
inputs$.pipe(
  takeLast(3)
).subscribe({
  next : (value) => {
    const item = document.createElement('div') ;
    item.textContent = `- ${valeur}` ;
    historyDisplay.appendChild(item) ;
  },.
  complete : () => {
    const note = document.createElement('div') ;
    note.style.marginTop = '5px' ;
    note.style.colour = 'grey' ;
    note.textContent = '(Reload the page to type again)' ;
    historyDisplay.appendChild(note) ;

    // Désactivation des champs de saisie et des boutons
    input.disabled = true ;
    submitButton.disabled = true ;
  }
}) ;

// Ajout d'une entrée avec la touche Entrée
fromEvent<KeyboardEvent>(input, 'keydown').subscribe(event => {
  if (event.key === 'Enter' && input.value.trim()) {
    inputs$.next(input.value) ;
    console.log(`Add : ${input.value}`) ;
    input.value = '' ;
  }
}) ;

// Compléter avec l'historique des clics et de l'affichage
fromEvent(submitButton, 'click').subscribe(() => {
  historyDisplay.innerHTML = '<strong>History (latest 3):</strong><br>' ;
  inputs$.complete() ; // complete stream → takeLast fires
}) ;

```

> [!IMPORTANT]
> **Points clés**:
> - `takeLast(3)` S'abonner à la**d'abord.**doit être mis en place en premier
> - lorsque l'on clique sur le bouton. `complete()` la dernière des valeurs reçues jusqu'à ce point sera éditée.3La dernière valeur reçue jusqu'à ce point est éditée.
> - `complete()` Après l'appel**Après avoir appelé**à `subscribe` les valeurs ne circulent pas.

## ⚠️ Un point important à noter

> [!WARNING]
> `takeLast` est d'attendre que le flux**Attendre l'achèvement**Par conséquent, cela ne fonctionne pas avec des flux infinis. De plus, la fonction`takeLast(n)` de la fonctionnest grande, elle consomme beaucoup de mémoire.

### 1. Il ne peut pas être utilisé avec des flux infinis.

`takeLast` ne fonctionne pas avec les flux infinis car il attend que le flux se termine.

```

ts.
import { interval } from 'rxjs' ;
import { takeLast } from 'rxjs' ;

// ❌ Mauvais exemple : utilisation de takeLast avec des flux infinis
interval(1000).pipe(
  takeLast(3)
).subscribe(console.log) ;.
// Rien n'est produit (parce que le flux ne se termine jamais)

```

**Solution.**: `take` Utilisez un flux fini en combinaison avec

```

ts.
import { interval } from 'rxjs' ;
import { take, takeLast } from 'rxjs' ;

// ✅ Bon exemple : flux fini puis utilisation de takeLast
interval(1000).pipe(
  take(10), // Complète avec les 10 premiers
  takeLast(3) // prend les 3 derniers
).subscribe(console.log) ;.
// Sortie : 7, 8, 9

```

### 2. Attention à l'utilisation de la mémoire

`takeLast(n)` ne fonctionne pas avec les flux finis car il retient la dernière pièce dans la mémoire tampon, ce qui peut entraîner une perte de mémoire.npièce à conserver dans la mémoire tampon,nest grand, il consomme plus de mémoire.

```

ts.
import { range } from 'rxjs' ;
import { takeLast } from 'rxjs' ;

// ⚠️ Remarque : les grandes quantités de données sont conservées dans une mémoire tampon.
range(0, 1000000).pipe(
  takeLast(100000) // 100 000 enregistrements conservés en mémoire
).subscribe(console.log) ;.

```

## 🎯 last La différence entre l

```

ts.
import { range } from 'rxjs' ;
import { last, takeLast } from 'rxjs' ;

const numbers$ = range(0, 10) ;

// last : seulement le dernier
numbers$.pipe(
  last()
).subscribe(console.log) ;
// sortie : 9

// takeLast(1) : dernier (sortie en tant que valeur unique, pas en tant que tableau)
numbers$.pipe(
  takeLast(1)
).subscribe(console.log) ;.
// Sortie : 9

// takeLast(3) : dernier 3
numbers$.pipe(
  takeLast(3)
).subscribe(console.log) ;
// Sortie : 7, 8, 9

```

| l'opérateur | Nombre d'acquisitions | Spécification de la condition | Cas d'utilisation |
|---|---|---|---|
| `last()` | 1Nombre d'acquisitions | possible | Obtenir le dernier1Pièces ou la dernière pièce qui remplit la condition1Nombre d'acquisitions |
| `takeLast(n)` | nNombre d'acquisitions | Impossible | Obtenir le derniernIl suffit d'obtenir la dernière pièce qui remplit la condition. |

## 📋 Utilisation sûre du point de vue du type

TypeScript Il s'agit d'un exemple d'implémentation sûre du point de vue du type, qui utilise les génériques en

```

ts.
import { Observable, from } from 'rxjs' ;
import { takeLast } from 'rxjs' ;

interface Transaction {
  id : string ;
  amount : nombre ;
  timestamp : Date ;
  status : 'pending' | 'complete' | 'failed' ; }
}

function getRecentTransactions(
  transactions$ : Observable,.
  count : nombre
) : Observable {
  return transactions$.pipe(
    takeLast(count)
  ) ;
}

// Exemple d'utilisation
const transactions$ = from([.
  { id : '1', amount : 100, timestamp : new Date('2025-01-01'), status : 'complete' as const }
  { id : '2', amount : 200, timestamp : new Date('2025-01-02'), status : 'complete' as const }
  { id : '3', amount : 150, timestamp : new Date('2025-01-03'), status : 'pending' as const }
  { id : '4', amount : 300, timestamp : new Date('2025-01-04'), status : 'complete' as const }
  { id : '5', amount : 250, timestamp : new Date('2025-01-05'), status : 'failed' as const }
] en tant que Transaction[]) ;.

// Obtenir les trois transactions les plus récentes
getRecentTransactions(transactions$, 3).subscribe(tx => {
  console.log(`${tx.id} : ${tx.amount} yen (${tx.status})`) ;
}) ;
// Sortie :.
// 3 : 150 yen (en attente)
// 4 : 300 yen (complete)
// 5 : 250 yens (échec)

```

## 🔄 skip et takeLast la combinaison de

La partie centrale de la valeur est exclue et seul le dernier morceau peut être récupéré.NSeule la dernière peut être récupérée.

```

ts
import { range } from 'rxjs' ;
import { skip, takeLast } from 'rxjs' ;

const numbers$ = range(0, 10) ; // 0 à 9

// sauter les 5 premiers et prendre les 3 derniers restants
numbers$.pipe(
  skip(5), // skip 0, 1, 2, 3, 4
  takeLast(3) // prend les 3 derniers parmi les 5, 6, 7, 8, 9 restants
).subscribe(console.log) ;.
// Sortie : 7, 8, 9
```

## 🎓 Résumé

### Quand takeLast doit être utilisé.
- ✅ Si vous avez besoin des N dernières données d'un flux.
- ✅ Si vous voulez obtenir les N derniers journaux ou transactions
- ✅ Si la fin du flux est garantie
- Si vous souhaitez afficher un résumé ou les N premiers enregistrements de données

### Quand vous devez utiliser take.
- ✅ Si vous avez besoin des N premières données du flux
- ✅ Si vous voulez obtenir les résultats immédiatement
- ✅ Si vous voulez obtenir une partie d'un flux infini

### Notes.
- ⚠️ Ne peut pas être utilisé avec des flux infinis (car ils ne se terminent pas).
- ⚠️ Un grand n dans `takeLast(n)` consomme de la mémoire
- ⚠️ La sortie est compilée après achèvement (pas immédiatement)
- ⚠️ Doit souvent être combiné avec `take(n)` pour obtenir un flux fini.

## 🚀 Prochaine étape.

- **[take](. /take)** - apprendre à obtenir les n premières valeurs.
- **[last](. /last)** - apprendre à obtenir les 1 dernières valeurs.
- **[skip](. /skip)** - apprendre à sauter les N premières valeurs.
- **[filter](. /filter)** - apprendre à filtrer en fonction de conditions
- **[filtering-operator-practical-use-cases](. /practical-use-cases)** - apprendre à utiliser des cas d'utilisation réels
