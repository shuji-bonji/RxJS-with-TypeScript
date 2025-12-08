---
description: "takeLast est un opérateur de filtrage RxJS qui ne produit que les N dernières valeurs lorsque le flux Observable se termine. Idéal pour obtenir les N dernières entrées du journal, afficher les N premiers éléments sur un tableau de classement, ou afficher un résumé final des données à la fin. Ne peut pas être utilisé avec des flux infinis car il conserve les valeurs dans un tampon jusqu'à la fin."
---

# takeLast - Récupérer les N dernières valeurs

L'opérateur `takeLast` ne produit que les N dernières valeurs **au moment où le flux se termine**. Il conserve les valeurs dans un tampon jusqu'à la fin, puis les émet toutes ensemble.


## 🔰 Syntaxe de base et utilisation

```ts
import { range } from 'rxjs';
import { takeLast } from 'rxjs';

const numbers$ = range(0, 10); // 0 à 9

numbers$.pipe(
  takeLast(3)
).subscribe(console.log);
// Sortie: 7, 8, 9
```

**Flux d'opération** :
1. Le flux émet 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
2. Les 3 dernières valeurs sont conservées en interne dans le tampon
3. Le flux se termine
4. Les valeurs du tampon 7, 8, 9 sont émises dans l'ordre

[🌐 Documentation officielle RxJS - `takeLast`](https://rxjs.dev/api/operators/takeLast)


## 🆚 Comparaison avec take

`take` et `takeLast` ont des comportements opposés.

```ts
import { range } from 'rxjs';
import { take, takeLast } from 'rxjs';

const numbers$ = range(0, 10); // 0 à 9

// take: récupère les N premiers
numbers$.pipe(
  take(3)
).subscribe(console.log);
// Sortie: 0, 1, 2 (émis immédiatement)

// takeLast: récupère les N derniers
numbers$.pipe(
  takeLast(3)
).subscribe(console.log);
// Sortie: 7, 8, 9 (émis après la fin)
```

| Opérateur | Position | Moment d'émission | Comportement avant fin |
|---|---|---|---|
| `take(n)` | Les n premiers | Émission immédiate | Fin automatique après n éléments |
| `takeLast(n)` | Les n derniers | Émission groupée après fin | Conservation dans le tampon |


## 💡 Patterns d'utilisation typiques

1. **Récupérer les N dernières entrées de journal**
   ```ts
   import { from } from 'rxjs';
   import { takeLast } from 'rxjs';

   interface LogEntry {
     timestamp: number;
     level: 'info' | 'warn' | 'error';
     message: string;
   }

   const logs$ = from([
     { timestamp: 1, level: 'info' as const, message: 'Application démarrée' },
     { timestamp: 2, level: 'info' as const, message: 'Utilisateur connecté' },
     { timestamp: 3, level: 'warn' as const, message: 'Requête lente détectée' },
     { timestamp: 4, level: 'error' as const, message: 'Échec de connexion' },
     { timestamp: 5, level: 'info' as const, message: 'Nouvelle tentative réussie' },
   ] as LogEntry[]);

   // Récupère les 3 derniers logs
   logs$.pipe(
     takeLast(3)
   ).subscribe(log => {
     console.log(`[${log.level}] ${log.message}`);
   });
   // Sortie:
   // [warn] Requête lente détectée
   // [error] Échec de connexion
   // [info] Nouvelle tentative réussie
   ```

2. **Récupérer le top N d'un classement**
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
     // Supposé trié par score
   );

   // Récupère le top 3
   scores$.pipe(
     takeLast(3)
   ).subscribe(score => {
     console.log(`${score.player}: ${score.score}`);
   });
   // Sortie: Charlie: 200, Dave: 180, Eve: 220
   ```

3. **Résumé des N derniers éléments après traitement des données**
   ```ts
   import { interval } from 'rxjs';
   import { take, map, takeLast } from 'rxjs';

   // Simulation de données de capteur
   const sensorData$ = interval(100).pipe(
     take(20),
     map(i => ({
       id: i,
       temperature: 20 + Math.random() * 10
     }))
   );

   // Calcule la température moyenne des 5 derniers
   sensorData$.pipe(
     takeLast(5)
   ).subscribe({
     next: data => {
       console.log(`Données ${data.id}: ${data.temperature.toFixed(1)}°C`);
     },
     complete: () => {
       console.log('Récupération des 5 dernières données terminée');
     }
   });
   ```


## 🧠 Exemple de code pratique (historique de saisie)

Exemple d'affichage des 3 dernières valeurs saisies par l'utilisateur.

```ts
import { fromEvent, Subject } from 'rxjs';
import { takeLast } from 'rxjs';

// Création des éléments UI
const container = document.createElement('div');
document.body.appendChild(container);

const input = document.createElement('input');
input.placeholder = 'Entrez une valeur et appuyez sur Entrée';
container.appendChild(input);

const submitButton = document.createElement('button');
submitButton.textContent = 'Afficher l\'historique (3 derniers)';
container.appendChild(submitButton);

const historyDisplay = document.createElement('div');
historyDisplay.style.marginTop = '10px';
container.appendChild(historyDisplay);

// Subject pour conserver les valeurs saisies
const inputs$ = new Subject<string>();

// **Important**: configurer la souscription à takeLast en premier
inputs$.pipe(
  takeLast(3)
).subscribe({
  next: (value) => {
    const item = document.createElement('div');
    item.textContent = `・ ${value}`;
    historyDisplay.appendChild(item);
  },
  complete: () => {
    const note = document.createElement('div');
    note.style.marginTop = '5px';
    note.style.color = 'gray';
    note.textContent = '(Rechargez la page pour saisir à nouveau)';
    historyDisplay.appendChild(note);

    // Désactiver le champ de saisie et le bouton
    input.disabled = true;
    submitButton.disabled = true;
  }
});

// Ajouter l'entrée avec la touche Entrée
fromEvent<KeyboardEvent>(input, 'keydown').subscribe(event => {
  if (event.key === 'Enter' && input.value.trim()) {
    inputs$.next(input.value);
    console.log(`Ajouté: ${input.value}`);
    input.value = '';
  }
});

// Terminer et afficher l'historique au clic sur le bouton
fromEvent(submitButton, 'click').subscribe(() => {
  historyDisplay.innerHTML = '<strong>Historique (3 derniers) :</strong><br>';
  inputs$.complete(); // Terminer le flux → takeLast se déclenche
});
```

> [!IMPORTANT]
> **Points importants** :
> - La souscription à `takeLast(3)` doit être configurée **en premier**
> - Lorsque vous appelez `complete()` au clic, les 3 dernières valeurs reçues sont émises
> - Si vous faites `subscribe` **après** avoir appelé `complete()`, aucune valeur ne sera émise


## ⚠️ Points d'attention importants

> [!WARNING]
> `takeLast` **attend que le flux se termine**, il ne fonctionne donc pas avec les flux infinis. De plus, si n est grand dans `takeLast(n)`, cela consomme beaucoup de mémoire.

### 1. Ne peut pas être utilisé avec des flux infinis

`takeLast` attend la fin du flux, il ne fonctionne donc pas avec les flux infinis.

```ts
import { interval } from 'rxjs';
import { takeLast } from 'rxjs';

// ❌ Mauvais exemple: utiliser takeLast avec un flux infini
interval(1000).pipe(
  takeLast(3)
).subscribe(console.log);
// Rien ne s'affiche (le flux ne se termine jamais)
```

**Solution** : combiner avec `take` pour créer un flux fini

```ts
import { interval } from 'rxjs';
import { take, takeLast } from 'rxjs';

// ✅ Bon exemple: créer un flux fini puis utiliser takeLast
interval(1000).pipe(
  take(10),      // Terminer après les 10 premiers
  takeLast(3)    // Récupérer les 3 derniers
).subscribe(console.log);
// Sortie: 7, 8, 9
```

### 2. Attention à l'utilisation mémoire

`takeLast(n)` conserve les n derniers éléments dans le tampon, donc un n élevé consomme de la mémoire.

```ts
import { range } from 'rxjs';
import { takeLast } from 'rxjs';

// ⚠️ Attention: conservation de nombreuses données dans le tampon
range(0, 1000000).pipe(
  takeLast(100000) // 100 000 éléments en mémoire
).subscribe(console.log);
```


## 🎯 Différence avec last

```ts
import { range } from 'rxjs';
import { last, takeLast } from 'rxjs';

const numbers$ = range(0, 10);

// last: uniquement le dernier
numbers$.pipe(
  last()
).subscribe(console.log);
// Sortie: 9

// takeLast(1): le dernier (émis comme valeur unique, pas comme tableau)
numbers$.pipe(
  takeLast(1)
).subscribe(console.log);
// Sortie: 9

// takeLast(3): les 3 derniers
numbers$.pipe(
  takeLast(3)
).subscribe(console.log);
// Sortie: 7, 8, 9
```

| Opérateur | Nombre | Condition | Cas d'utilisation |
|---|---|---|---|
| `last()` | 1 | Possible | Le dernier ou le dernier satisfaisant une condition |
| `takeLast(n)` | n | Impossible | Simplement récupérer les n derniers |


## 📋 Utilisation type-safe

Exemple d'implémentation type-safe avec les génériques TypeScript.

```ts
import { Observable, from } from 'rxjs';
import { takeLast } from 'rxjs';

interface Transaction {
  id: string;
  amount: number;
  timestamp: Date;
  status: 'pending' | 'completed' | 'failed';
}

function getRecentTransactions(
  transactions$: Observable<Transaction>,
  count: number
): Observable<Transaction> {
  return transactions$.pipe(
    takeLast(count)
  );
}

// Exemple d'utilisation
const transactions$ = from([
  { id: '1', amount: 100, timestamp: new Date('2025-01-01'), status: 'completed' as const },
  { id: '2', amount: 200, timestamp: new Date('2025-01-02'), status: 'completed' as const },
  { id: '3', amount: 150, timestamp: new Date('2025-01-03'), status: 'pending' as const },
  { id: '4', amount: 300, timestamp: new Date('2025-01-04'), status: 'completed' as const },
  { id: '5', amount: 250, timestamp: new Date('2025-01-05'), status: 'failed' as const },
] as Transaction[]);

// Récupère les 3 dernières transactions
getRecentTransactions(transactions$, 3).subscribe(tx => {
  console.log(`${tx.id}: ${tx.amount}€ (${tx.status})`);
});
// Sortie:
// 3: 150€ (pending)
// 4: 300€ (completed)
// 5: 250€ (failed)
```


## 🔄 Combinaison de skip et takeLast

Vous pouvez exclure les valeurs intermédiaires et récupérer uniquement les N dernières.

```ts
import { range } from 'rxjs';
import { skip, takeLast } from 'rxjs';

const numbers$ = range(0, 10); // 0 à 9

// Ignorer les 5 premiers et récupérer les 3 derniers du reste
numbers$.pipe(
  skip(5),      // Ignorer 0, 1, 2, 3, 4
  takeLast(3)   // Les 3 derniers parmi 5, 6, 7, 8, 9
).subscribe(console.log);
// Sortie: 7, 8, 9
```


## 🎓 Résumé

### Quand utiliser takeLast
- ✅ Lorsque vous avez besoin des N dernières données du flux
- ✅ Lorsque vous voulez récupérer les N dernières entrées de journal ou transactions
- ✅ Lorsque la fin du flux est garantie
- ✅ Lorsque vous voulez afficher un résumé ou le top N

### Quand utiliser take
- ✅ Lorsque vous avez besoin des N premières données du flux
- ✅ Lorsque vous voulez obtenir des résultats immédiatement
- ✅ Lorsque vous voulez récupérer une partie d'un flux infini

### Points d'attention
- ⚠️ Ne peut pas être utilisé avec des flux infinis (ne se termine jamais)
- ⚠️ Un n élevé dans `takeLast(n)` consomme de la mémoire
- ⚠️ L'émission est groupée après la fin (pas d'émission immédiate)
- ⚠️ Souvent nécessaire de combiner avec `take(n)` pour créer un flux fini


## 🚀 Prochaines étapes

- **[take](./take)** - Apprendre à récupérer les N premières valeurs
- **[last](./last)** - Apprendre à récupérer la dernière valeur
- **[skip](./skip)** - Apprendre à ignorer les N premières valeurs
- **[filter](./filter)** - Apprendre le filtrage basé sur les conditions
- **[Exemples pratiques d'opérateurs de filtrage](./practical-use-cases)** - Apprendre des cas d'utilisation réels
