---
description: "zipWith é um operador de junção RxJS que alinha e emparelha o Observable original com os valores ordenados correspondentes de outros Observables. É ideal para junções de fluxo com ordem crítica, como a combinação de resultados de processamento paralelo com garantias de ordem, mapeamento de IDs para dados, sincronização de dados relacionados publicados em momentos diferentes etc. A versão do operador pipe é conveniente para uso em pipelines."
---

# zipWith - par de valores correspondentes

O operador `zipWith` agrupa os **valores ordenados correspondentes** emitidos pelo Observable original e pelos outros Observables especificados.
Ele aguarda até que os valores cheguem um a um de todos os Observable e cria pares quando eles estiverem prontos.
Essa é a versão do Pipeable Operator do `zip` da Creation Function.

## Sintaxe básica e uso

```ts
import { of, interval } from 'rxjs';
import { zipWith, map, take } from 'rxjs';

const letters$ = of('A', 'B', 'C', 'D');
const numbers$ = interval(1000).pipe(
  map(val => val * 10),
  take(3)
);

letters$
  .pipe(zipWith(numbers$))
  .subscribe(([letter, number]) => {
    console.log(`${letter} - ${number}`);
  });

// Saída:
// A - 0
// B - 10
// C - 20
// (Dnão é uma saída, pois não há um valor correspondente)
```

- Um par é gerado a partir de cada Observable **quando um dos valores é concluído**.
- Quando um Observable é concluído, os valores restantes são descartados.

[🌐 Documentação oficial do RxJS - zipWith](https://rxjs.dev/api/operators/zipWith)

## 💡 Padrão de utilização típico.

- **Combinação de resultados de processamento paralelo em ordem garantida**: emparelhamento dos resultados de várias chamadas de API.
- **Mapeamento de IDs para dados**: combinação de IDs de usuário com dados de perfil correspondentes
- Sincronização de fluxos**: sincronização de dados relacionados emitidos em momentos diferentes

## Exemplos práticos de código (com UI)

Exemplo de emparelhamento de uma lista de IDs de usuário e nomes de usuário correspondentes em sequência.

```ts
import { from, of } from 'rxjs';
import { zipWith, delay, concatMap } from 'rxjs';

// Criação de área de saída
const output = document.createElement('div');
output.innerHTML = '<h3>zipWith Exemplos práticos de:</h3>';
document.body.appendChild(output);

// UsuárioIDStream (publicado imediatamente)
const userIds$ = from([101, 102, 103, 104]);

// Fluxo de nome de usuário (emitido a cada1(emitido a cada segundo)
const userNames$ = from(['Alice', 'Bob', 'Carol']).pipe(
  concatMap(name => of(name).pipe(delay(1000)))
);

// zipe exibido
userIds$
  .pipe(zipWith(userNames$))
  .subscribe(([id, name]) => {
    const item = document.createElement('div');
    item.textContent = `👤 UsuárioID ${id}: ${name}`;
    output.appendChild(item);
  });

// Saída:
// 👤 UsuárioID 101: Alice
// 👤 UsuárioID 102: Bob
// 👤 UsuárioID 103: Carol
// (104não é exibido porque não há um nome correspondente)
```

- As IDs e os nomes são emparelhados em uma correspondência de **um para um**.
- Quando um é completado, os valores restantes são descartados.

## Diferença da Creation Function `zip`.

### Diferenças básicas.

|  | `zip` (Creation Function). | `zipWith` (Pipeable Operator) |
|---|---|---|
| **Onde é usado** | Usado como uma função autônoma | Usada na cadeia `.pipe()`. |
| **Como escrever**. | `zip(obs1$, obs2$, obs3$)` | `obs1$.pipe(zipWith(obs2$, obs3$))` |
| **Primeiro fluxo**. | Tratar todos como iguais | Tratar como fluxo principal |
| **Vantagens**. | Simples e fácil de ler | Fácil de combinar com outros operadores |

### Exemplos específicos de uso

**Para emparelhamento simples, recomenda-se a Creation Function**.

```ts
import { zip, of } from 'rxjs';

const questions$ = of('O nome é？', 'A idade é？', 'O endereço é？');
const answers$ = of('Taro', '30', 'Tóquio');
const scores$ = of(10, 20, 30);

// Simples e legível
zip(questions$, answers$, scores$).subscribe(([q, a, s]) => {
  console.log(`Q: ${q}, A: ${a}, Pontuação: ${s}Pontuação`);
});
// Saída:
// Q: O nome é？, A: Taro, Pontuação: 10Pontuação
// Q: A idade é？, A: 30, Pontuação: 20Pontuação
// Q: O endereço é？, A: Tóquio, Pontuação: 30Pontuação
```

**Se você quiser adicionar um processo de transformação ao fluxo principal, recomenda-se o Pipeable Operator**.

```ts
import { from, interval } from 'rxjs';
import { zipWith, map, take, filter } from 'rxjs';

// Lista de tarefas
const tasks$ = from([
  { id: 1, name: 'Relatórios', priority: 'high' },
  { id: 2, name: 'Resposta a e-mails', priority: 'low' },
  { id: 3, name: 'Preparação de reuniões', priority: 'high' },
  { id: 4, name: 'Organizar documentos', priority: 'medium' }
]);

// Lista de pessoas responsáveis (1Atribuído a cada segundo)
const assignees$ = from(['Alice', 'Bob', 'Carol', 'Dave']).pipe(
  zipWith(interval(1000).pipe(take(4))),
  map(([name]) => name)
);

// ✅ Pipeable OperatorEdições - Concluídas em um único pipeline
tasks$
  .pipe(
    filter(task => task.priority === 'high'),  // Somente prioridade alta
    map(task => task.name),                     // Extrair nomes de tarefas
    zipWith(assignees$),                        // Atribuir pessoa responsável
    map(([taskName, assignee]) => ({
      task: taskName,
      assignee,
      assignedAt: new Date().toLocaleTimeString()
    }))
  )
  .subscribe(assignment => {
    console.log(`[${assignment.assignedAt}] ${assignment.task} → Atribuído a: ${assignment.assignee}`);
  });
// Saída:
// [Tempo] Relatórios → Atribuído a: Alice
// [Tempo] Preparação de reuniões → Atribuído a: Bob

// ❌ Creation FunctionEdições - Redundante
import { zip } from 'rxjs';
zip(
  tasks$.pipe(
    filter(task => task.priority === 'high'),
    map(task => task.name)
  ),
  assignees$
).pipe(
  map(([taskName, assignee]) => ({
    task: taskName,
    assignee,
    assignedAt: new Date().toLocaleTimeString()
  }))
).subscribe(assignment => {
  console.log(`[${assignment.assignedAt}] ${assignment.task} → Atribuído a: ${assignment.assignee}`);
});
```

**Sincronização de dados críticos para a ordem**.

```ts
import { from } from 'rxjs';
import { zipWith, map, concatMap, delay } from 'rxjs';
import { of } from 'rxjs';

// UICriar
const output = document.createElement('div');
output.innerHTML = '<h3>Jogo de perguntas</h3>';
document.body.appendChild(output);

const questionArea = document.createElement('div');
questionArea.style.marginTop = '10px';
output.appendChild(questionArea);

// Lista de perguntas (preparada imediatamente)
const questions$ = from([
  'A capital do Japão é？',
  '1+1é？',
  'Que planeta é a Terra?？'
]);

// Lista de respostas (simulando a entrada do usuário)：2(a cada segundo)
const answers$ = from(['Tóquio', '2', '3']).pipe(
  concatMap((answer, index) =>
    of(answer).pipe(delay((index + 1) * 2000))
  )
);

// Lista de respostas corretas
const correctAnswers$ = from(['Tóquio', '2', '3']);

// ✅ Pipeable OperatorEdições - Processar perguntas como mainstream
questions$
  .pipe(
    zipWith(answers$, correctAnswers$),
    map(([question, answer, correct], index) => ({
      no: index + 1,
      question,
      answer,
      correct,
      isCorrect: answer === correct
    }))
  )
  .subscribe(result => {
    const div = document.createElement('div');
    div.style.marginTop = '10px';
    div.style.padding = '10px';
    div.style.border = '1px solid #ccc';
    div.style.backgroundColor = result.isCorrect ? '#e8f5e9' : '#ffebee';
    div.innerHTML = `
      <strong>Perguntas${result.no}:</strong> ${result.question}<br>
      <strong>Respostas:</strong> ${result.answer}<br>
      <strong>Resultado:</strong> ${result.isCorrect ? '✅ Resposta correta！' : '❌ Resposta incorreta'}
    `;
    questionArea.appendChild(div);
  });
```

### Resumo.

- zip: melhor se você quiser apenas mapear vários fluxos em ordem
- zip: ideal se você quiser mesclar o fluxo principal com outros fluxos em ordem garantida enquanto transforma ou processa o fluxo principal

## ⚠️ Notas.

### Para comprimentos diferentes.

Quando o Observable mais curto é concluído, os valores restantes do mais longo são descartados.

```ts
import { of } from 'rxjs';
import { zipWith } from 'rxjs';

const short$ = of(1, 2, 3);
const long$ = of('A', 'B', 'C', 'D', 'E');

short$.pipe(zipWith(long$)).subscribe(console.log);
// Saída: [1, 'A'], [2, 'B'], [3, 'C']
// 'D'e'E'são descartados
```

### Acumulação de memória.

Se um Observable continuar a emitir valores, os valores se acumularão na memória até que o outro os recupere.

```ts
import { interval} from 'rxjs';
import { zipWith, take } from 'rxjs';

// Fluxos rápidos (100mspor fluxo)
const fast$ = interval(100).pipe(take(10));

// Fluxo de baixa velocidade (1(a cada segundo)
const slow$ = interval(1000).pipe(take(3));

fast$.pipe(zipWith(slow$)).subscribe(console.log);
// Saída: [0, 0] (1segundos depois), [1, 1] (2segundos depois), [2, 2] (3segundos depois)
// fast$Os valores são armazenados na memória e aguardados
```

### Diferença de combineLatestWith.

O `zipWith` emparelha os valores na ordem correspondente, enquanto o `combineLatestWith` combina os valores mais recentes.

```ts
import { interval } from 'rxjs';
import { zipWith, combineLatestWith, take } from 'rxjs';

const source1$ = interval(1000).pipe(take(3));
const source2$ = interval(1500).pipe(take(2));

// zipWith: Emparelhados na ordem correspondente
source1$.pipe(zipWith(source2$)).subscribe(console.log);
// Saída: [0, 0], [1, 1]

// combineLatestWith: Combinar os valores mais recentes
source1$.pipe(combineLatestWith(source2$)).subscribe(console.log);
// Saída: [0, 0], [1, 0], [2, 0], [2, 1]
```

## 📚 Operadores relacionados.

- **[zip](/pt/guide/creation-functions/combination/zip)** - Versão da Creation Function.
- **[combineLatestWith](/pt/guide/operators/combination/combineLatestWith)** - Combina o valor mais recente
- **[withLatestFrom](/pt/guide/operators/combination/withLatestFrom)** - somente acionadores principais
