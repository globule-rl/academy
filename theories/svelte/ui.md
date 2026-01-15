<!-- binding: text input -->
let name = $state('');
<input bind:value={name} placeholder="enter name" />
<p>Hello {name || 'stranger'}.</p>

<!-- binding: value -->
<script>
    let a = $state(1);
 </script>
 <label for="">
    <input type="range" bind:value={a} min="0" max="10" />
 </label>
 <p>{a}</p>

 <!-- binding: checkbox -->
<script>
    let yes = $state(false);
</script>
<label>
<input type="checkbox" bind:checked={yes} />
</label>
{#if yes}
    <p> </p>
{:else}
    <p> </p>
{/if}
<button disable={!yes}> Sub </button>

return `${flavors.slice(0, -1).join(', ')} and ${flavors[flavors.length-1]}`;

let text = $state(`*italic*, **bold**`);
{@html marked(text)}
const safeHtml = DOMPurify.sanitize(marked(text));
{@html safeHtml};

<!-- file input -->
<script>
let files = $state();
// effect() -> side effects: log to console, send data to server, trigger animation, 
// outside of sv reactivity DOM/local
$effect(() =>
{
    if (files)
    {
        console.log(files);
        for (const file of files)
        {
            console.log(`${file.name}: ${file.size} bytes`);
        }
    }
});
</script>
<input accept="image/png, image/jpeg" bind:files id="avatar" name="avatar" type="file" />
<input bind:files id="many" multiple type="file" />
<!-- render DOM  content to user-->
{#if files}
    {#each Array.from(files) as file}
        <p>{file.name} ({file.size} bytes)</p>
    {/each}
{/if}

<!-- select binding -->
<!-- handleSubmit has both selected and answer -->
<form onsubmit={handleSubmit}>
    <!-- clear the old answer -->
    <select bind:value={selected} onchange={() => (answer = '')}>
    <select multiple bind:value={flavors}>
        {#each questions as q}
            <option value={q}>
                {q.text}
            </option>
        {/each}
    </select>
    <!-- bind the option to 'selected' -->
    <input bind:value={answer}>
    <button disable={!answer} type="submit">Submit</button>
</form>

<!-- each binding -->
let todos = $state([
    {done: false, text: 'todo1' },
]);
function add()
{
    todos = todos.concat({ done: false, text: '' });
}
function clear()
{
    todos = todos.filter((t) => !t.done);
}
let remaining = $derived(todos.filter((t) => !t.done).length);
{#each todos as todo}
    <div>
        <input type="checkbox" bind:checked={todo.done} />
        <input placeholder="todo" bind:value={todo.text} disabled={todo.done} />
    </div>
{/each}